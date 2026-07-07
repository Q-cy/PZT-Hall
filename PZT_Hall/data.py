"""
数据采集模块
功能：压力传感器/六维力传感器串口读取、解码、缓存、重连
"""
import serial
import serial.tools.list_ports
import time
import struct
from collections import deque
import threading
import numpy as np

DATA_BAUDRATE_PRESS = 921600  # 压力传感器串口波特率
DATA_BAUDRATE_FORCE = 460800  # 六维力传感器串口波特率

# ===================== 压力传感器 =====================
class PressureSensor:                              # 为什么定义成类而不是函数？1.包含很多函数，2.方便管理状态，有很多global变量
    def __init__(self):
        self.ser = None                            # self.变量是默认global
        self.port = None
        self.auto_find_port()

    def auto_find_port(self):
        """固定使用 /dev/ttyUSB0"""
        try:
            self.ser = serial.Serial("/dev/ttyUSB0", DATA_BAUDRATE_PRESS, timeout=0.01)
            self.port = "/dev/ttyUSB0"
            time.sleep(0.1)
            self.ser.reset_input_buffer()
            return
        except:
            raise Exception("未找到压力传感器(/dev/ttyUSB0)")

    def reconnect(self):
        """断开重连"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except:
            pass
        time.sleep(0.2)
        self.auto_find_port()

    @staticmethod
    def crc8_itu(data: bytes) -> int:
        """CRC-8-ITU 校验（多项式 0x07，初始值 0x00）"""
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc ^ 0x55

    def read_data(self):
        """读取一帧原始数据：write 后循环读满一帧，丢弃多余数据"""
        if not self.ser or not self.ser.is_open:
            return None
        try:
            self.ser.reset_input_buffer()
            cmd = [0x55, 0xAA, 0x09, 0x00, 0x34, 0x00, 0xFB, 0x00, 0x1C, 0x00, 0x00, 0xBC, 0x00, 0x36]
            self.ser.write(bytearray(cmd))
            # 循环读取直到收满一帧（203 字节），超时 50ms
            resp = b''
            t0 = time.perf_counter()
            while len(resp) < 203 and time.perf_counter() - t0 < 0.05:
                chunk = self.ser.read(256)
                if chunk:
                    resp += chunk

            if not resp:
                return None
            idx = resp.find(b'\xaa\x55')
            if idx == -1 or len(resp[idx:]) < 203:
                return None

            frame = resp[idx:idx+203]

            # 状态字节检查：byte[13] == 0 表示成功
            if frame[13] != 0:
                return None

            # CRC-8-ITU 校验：byte[0:202] 的 CRC 与 byte[202] 比对
            if self.crc8_itu(frame[:202]) != frame[202]:
                return None

            return frame[14:202]   # 188 字节数据（156B PZT + 32B Hall）
        except Exception:
            return None

    def decode(self, raw):
        """解码为 PZT(78通道) + Hall(4传感器)"""
        # PZT: 前156字节 → 78个uint16，按13行×6列重排
        pzt_data = [struct.unpack("<H", raw[i:i+2])[0] for i in range(0, 156, 2)]
        out = []
        for i in range(13):
            out.extend(pzt_data[i*6:(i+1)*6])

        # Hall: 后32字节 → 4个TMAG3001传感器，每个8字节(小端int16)
        # 顺序：hall2(164), hall4(180), hall1(156), hall3(172)
        def _decode_hall(off):
            temp_raw = struct.unpack("<h", raw[off:off+2])[0]
            mag_x_raw = struct.unpack("<h", raw[off+2:off+4])[0]
            mag_y_raw = struct.unpack("<h", raw[off+4:off+6])[0]
            mag_z_raw = struct.unpack("<h", raw[off+6:off+8])[0]
            return {
                "temp_c": 25.0 + (temp_raw - 17512) / 58.2,
                "mag_x": mag_x_raw / 885.0 * 10.0,
                "mag_y": mag_y_raw / 885.0 * 10.0,
                "mag_z": mag_z_raw / 885.0 * 10.0,
            }
        hall_data = [_decode_hall(164), _decode_hall(180), _decode_hall(156), _decode_hall(172)]

        return out, hall_data

# ===================== 六维力传感器 =====================
class SixAxisForceSensor:
    def __init__(self):
        self.ser = None
        self.port = None
        self.zero_data = [0.0]*6
        self.auto_find_port()

    def auto_find_port(self):
        """自动寻找可用串口（排除 /dev/ttyUSB0）"""
        ports = list(serial.tools.list_ports.comports())
        for p, _, _ in ports:
            if p == "/dev/ttyUSB0":
                continue
            try:
                self.ser = serial.Serial(p, DATA_BAUDRATE_FORCE, timeout=0.05)
                self.port = p
                time.sleep(0.1)
                self.ser.reset_input_buffer()
                return
            except:
                continue
        raise Exception("未找到六维力传感器")

    def open_port(self):
        try:
            self.ser = serial.Serial(self.port, DATA_BAUDRATE_FORCE, timeout=0.05)
            time.sleep(0.1)
            self.ser.reset_input_buffer()
        except:
            self.ser = None

    def reconnect(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except:
            pass
        time.sleep(0.2)
        self.open_port()

    def read(self):
        """读取力/力矩数据（清空缓存 + 帧头校验）"""
        if not self.ser or not self.ser.is_open:
            return None
        try:
            self.ser.reset_input_buffer()  # 清空残留，防帧错位
            self.ser.write(b'\x49\xAA\x0D\x0A')
            time.sleep(0.008)
            resp = self.ser.read(28)
            if len(resp) != 28 or resp[:2] != b'\x49\xAA' or resp[-2:] != b'\x0D\x0A':
                return None
            Fx = struct.unpack('<f', resp[2:6])[0]
            Fy = struct.unpack('<f', resp[6:10])[0]
            Fz = struct.unpack('<f', resp[10:14])[0]
            Mx = struct.unpack('<f', resp[14:18])[0]
            My = struct.unpack('<f', resp[18:22])[0]
            Mz = struct.unpack('<f', resp[22:26])[0]
            Fx *= 9.8; Fy *= 9.8; Fz *= 9.8
            Mx *= 9.8; My *= 9.8; Mz *= 9.8
            Fx -= self.zero_data[0]; Fy -= self.zero_data[1]; Fz -= self.zero_data[2]
            Mx -= self.zero_data[3]; My -= self.zero_data[4]; Mz -= self.zero_data[5]
            return [round(v, 2) for v in [Fx, Fy, Fz, Mx, My, Mz]]
        except Exception as e:
            return None

    def calibrate_zero(self):
        """零点校准"""
        vals = []
        for _ in range(20):
            d = self.read()
            if d:
                vals.append(d)
            time.sleep(0.05)
        if len(vals) >= 5:
            self.zero_data = np.mean(np.array(vals), axis=0).tolist()

# ===================== 带时间戳的线程安全缓存 =====================
class TimestampedBuffer:
    def __init__(self, maxlen=500):
        self.buf = deque(maxlen=maxlen)
        self.lock = threading.Lock()

    def append(self, item):
        with self.lock:
            self.buf.append(item)

    def get_latest(self):
        with self.lock:
            return self.buf[-1] if self.buf else None

    def find_closest(self, ts):
        with self.lock:
            best = None
            best_dt = 1e9
            for item in self.buf:
                dt = abs(item["t"] - ts)
                if dt < best_dt:
                    best_dt = dt
                    best = item
            return best