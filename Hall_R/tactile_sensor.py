"""
触觉传感器数据采集模块
功能：读取 n Hall信号和 r*c 压阻信号
"""

from collections import deque
import serial
import struct
import threading
import time


# ==================== UART Config ====================

TACTILE_SENSOR_PORT = "/dev/ttyUSB0"
TACTILE_SENSOR_DATA_BAUDRATE = 921600
TACTILE_SENSOR_REQUEST_INTERVAL = 0.005
TACTILE_SENSOR_RESPONSE_INTERVAL = 0.001

# ==================== Sensor Config ====================

PIEZORESISTANCE_ROWS = 13
PIEZORESISTANCE_COLS = 6
HALL_SENSOR_COUNT = 4
HALL_SENSOR_LEN = 8


# ==================== Protocol Config====================
PIEZORESISTANCE_CELL_COUNT = PIEZORESISTANCE_ROWS * PIEZORESISTANCE_COLS
PIEZORESISTANCE_PAYLOAD_LEN = PIEZORESISTANCE_CELL_COUNT * 2

HALL_PAYLOAD_LEN = HALL_SENSOR_COUNT * HALL_SENSOR_LEN

RESPONSE_PAYLOAD_LEN = PIEZORESISTANCE_PAYLOAD_LEN + HALL_PAYLOAD_LEN

assert PIEZORESISTANCE_ROWS > 0
assert PIEZORESISTANCE_COLS > 0
assert HALL_SENSOR_COUNT >= 0
assert 0 < RESPONSE_PAYLOAD_LEN <= 0xFFFF


# ==================== CRC ====================

def crc8_itu(data: bytes) -> int:
    """
    CRC-8-ITU
    多项式：0x07
    初始值：0x00
    最终异或：0x55
    """
    crc = 0x00

    for byte in data:
        crc ^= byte

        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF

    return crc ^ 0x55


# ==================== Request Protocol ====================

class REQUEST_PROTOCOL:
    """
    串口读操作请求帧

    data[0:1]   起始符
    data[2:3]   请求数据长度
    data[4]     设备地址
    data[5]     预留
    data[6]     功能码
    data[7:10]  起始地址
    data[11:12] 读取数据长度
    data[13]    CRC-8-ITU
    """

    START_BYTES = b"\x55\xAA"         # b代表bytes，\x代表0x，但不能\0x，因为\0代表0，0x55代表int

    REQUEST_CONFIG_LEN = 9            # data[4-12]

    REQUEST_DATA_LEN_BYTES = struct.pack("<H", REQUEST_CONFIG_LEN)
    DEVICE_ADDRESS = 0x34
    RESERVED = 0x00
    FUNCTION_CODE = 0xFB
    START_ADDRESS = 0x00001C00
    START_ADDRESS_BYTES = struct.pack("<I", START_ADDRESS)            # I 无符号32位整数， H 16位
    READ_DATA_LEN_BYTES = struct.pack("<H", RESPONSE_PAYLOAD_LEN)

    def __init__(self):

        self.request_frame = self.build_request_frame()

    def build_request_frame(self):

        request_frame_withoutcrc = (
            self.START_BYTES 
            + self.REQUEST_DATA_LEN_BYTES 
            + bytes([self.DEVICE_ADDRESS, self.RESERVED, self.FUNCTION_CODE])
            + self.START_ADDRESS_BYTES
            + self.READ_DATA_LEN_BYTES
            )

        request_frame_crc = crc8_itu(request_frame_withoutcrc)

        request_frame_bytes = request_frame_withoutcrc + bytes([request_frame_crc])

        return request_frame_bytes


# ==================== RESPONSE Protocol ====================

class RESPONSE_PROTOCOL:
    """
    串口读操作回复帧

    data[0:1]       起始符
    data[2:3]       回复数据长度
    data[4]         设备地址
    data[5]         预留
    data[6]         功能码
    data[7:10]      起始地址
    data[11:12]     返回字节数N
    data[13]        状态
    data[14:N+13]   N字节传感器数据
    data[N+14]      CRC-8-ITU
    """
    
    START_BYTES_LEN = 2
    START_BYTES = b"\xAA\x55"       # data[0:1]
    RESPONSE_DATA_LEN_LEN = 2
    DEVICE_ADDRESS_LEN = 1
    DEVICE_ADDRESS = 0x34           # data[4]
    RESERVED_LEN = 1
    RESERVED = 0x00                 # data[5]
    FUNCTION_CODE_LEN = 1
    FUNCTION_CODE = 0xFB            # data[6]
    START_ADDRESS_LEN = 4
    START_ADDRESS = 0x00001C00      
    START_ADDRESS_BYTES = struct.pack("<I", START_ADDRESS)               # data[7:10]
    RESPONSE_BYTES_NUM_LEN = 2 
    RESPONSE_BYTES_NUM_BYTES = struct.pack("<H", RESPONSE_PAYLOAD_LEN)   # data[11:12]
    STATE_LEN = 1
    STATE_SUCCESS = 0x00            # data[13]
    RESPONSE_CRC_LEN = 1

    RESPONSE_FRAME_HEADER_LEN = START_BYTES_LEN + RESPONSE_DATA_LEN_LEN 
    RESPONSE_CONFIG_LEN = DEVICE_ADDRESS_LEN + RESERVED_LEN + FUNCTION_CODE_LEN + START_ADDRESS_LEN + RESPONSE_BYTES_NUM_LEN + STATE_LEN
    RESPONSE_DATA_LEN = RESPONSE_CONFIG_LEN + RESPONSE_PAYLOAD_LEN           
    RESPONSE_DATA_LEN_BYTES = struct.pack("<H", RESPONSE_DATA_LEN)       # data[2:3]

    RESPONSE_FRAME_LEN = RESPONSE_FRAME_HEADER_LEN + RESPONSE_CONFIG_LEN + RESPONSE_PAYLOAD_LEN + RESPONSE_CRC_LEN

    PAYLOAD_START = RESPONSE_FRAME_HEADER_LEN + RESPONSE_CONFIG_LEN
    PAYLOAD_END = PAYLOAD_START + RESPONSE_PAYLOAD_LEN

    CRC_START = PAYLOAD_END

    def decode_response(self, response_frame):

        # 检查输入类型
        if not isinstance(response_frame, bytes):
            raise TypeError(
                "response_frame must be bytes"
            )

        # 检查完整帧长度
        if len(response_frame) != self.RESPONSE_FRAME_LEN:
            raise ValueError(
                f"response len error："
                f"received {len(response_frame)}bytes，"
                f"should be{self.RESPONSE_FRAME_LEN}bytes"
            )

        # 检查data[0:1]起始符
        if response_frame[0:2] != self.START_BYTES:
            raise ValueError(
                "start bytes error"
            )

        # 检查data[2:3]回复数据长度
        if response_frame[2:4] != self.RESPONSE_DATA_LEN_BYTES:
            raise ValueError(
                "response data len error"
            )

        # 检查data[4]设备地址
        if response_frame[4] != self.DEVICE_ADDRESS:
            raise ValueError(
                "device address error"
            )

        # 检查data[5]预留字节
        if response_frame[5] != self.RESERVED:
            raise ValueError(
                "resverved byte error"
            )

        # 检查data[6]功能码
        if response_frame[6] != self.FUNCTION_CODE:
            raise ValueError(
                "function code error"
            )

        # 检查data[7:10]起始地址
        if response_frame[7:11] != self.START_ADDRESS_BYTES:
            raise ValueError(
                "start address error"
            )

        # 检查data[11:12]返回字节数
        if response_frame[11:13] != self.RESPONSE_BYTES_NUM_BYTES:
            raise ValueError(
                "response data num error"
            )

        # 检查data[13]状态
        if response_frame[13] != self.STATE_SUCCESS:
            raise ValueError(
                f"state error："
                f"0x{response_frame[13]:02X}"
            )

        # 读取回复帧中的CRC
        response_crc = response_frame[
            self.CRC_START
        ]

        # 重新计算CRC，不包含最后的CRC字节
        calculated_crc = crc8_itu(
            response_frame[:self.CRC_START]
        )

        if response_crc != calculated_crc:
            raise ValueError(
                f"CRC error："
                f"recived 0x{response_crc:02X}，"
                f"calculated 0x{calculated_crc:02X}"
            )

        # 提取data[14:N+13]传感器原始负载
        response_payload = response_frame[
            self.PAYLOAD_START:self.PAYLOAD_END
        ]

        return response_payload


# ==================== UART control ====================
class TactileSensor:
    def __init__(
        self,
        port = TACTILE_SENSOR_PORT,
        baudrate = TACTILE_SENSOR_DATA_BAUDRATE,
    ):
        self.port = port
        self.baudrate = baudrate
        self.serial = None

        self.request_protocol = REQUEST_PROTOCOL()
        self.response_protocol = RESPONSE_PROTOCOL()

        self._rx_buffer = bytearray()                   # 原始字节流，可变字节数组，自动扩容，按索引访问
        self._frame_buffer = deque(maxlen = 256)        # 解析验证通过的数据流，双端队列，按FIFO顺序访问, 不适合通过索引访问且不支持切片

        self._buffer_lock = threading.Lock()            
        self._running = threading.Event()               # 控制发送和读取线程是否继续运行, 初始为False

        self._request_thread = None
        self._response_thread = None
        self._start_time_ns = None
        self.last_error = None

        self.open_port()

        self.start()

    def open_port(self):

        self.serial = serial.Serial(
            port = self.port,
            baudrate = self.baudrate,
            timeout = 0.01,
        )

        # 等待串口初始化完成
        time.sleep(0.1)

        # 清除设备连接前残留的接收数据
        self.serial.reset_input_buffer()

    def start(self):
        
        # 防止重复启动线程
        if self._running.is_set():                      
            return                           

        # 记录采集启动时刻，后续时间戳从0ms开始
        self._start_time_ns = time.perf_counter_ns()
        self._running.set()                          # 打开开关
        self._request_thread = threading.Thread(     # 创建线程对象
            target = self._request_loop,             # 加括号表示现在立即执行函数, 不加括号表示把“函数本身”交给线程
            daemon = True,                           # Python允许进程直接退出
            )

        self._response_thread = threading.Thread(
            target = self._response_loop,
            daemon = True,
        )

        self._response_thread.start()                # 先启动接收线程，再启动发送线程
        self._request_thread.start()   

    def _request_loop(self):

        while self._running.is_set():
            self.serial.write(self.request_protocol.request_frame)
            time.sleep(TACTILE_SENSOR_REQUEST_INTERVAL)

    def _response_loop(self):

        while self._running.is_set():
            waiting_bytes = self.serial.in_waiting  # 查询操作系统串口驱动接收缓冲区当前有多少字节
            if waiting_bytes <= 0:
                time.sleep(TACTILE_SENSOR_RESPONSE_INTERVAL)
                continue

            received_data = self.serial.read(waiting_bytes)

            if received_data:
                with self._buffer_lock:             # with 是 Python 的“上下文管理”语句，用于保证某种资源在使用后自动释放
                    self._rx_buffer.extend(received_data)
                    self._parse_response_buffer()

    def _parse_response_buffer(self):
        """从原始字节缓存中提取并校验完整回复帧"""

        while True:
            # 在原始字节流中寻找回复帧头AA 55
            frame_start = self._rx_buffer.find(
                self.response_protocol.START_BYTES
            )

            # 没有找到完整帧头
            if frame_start < 0:
                # 如果最后一个字节是AA，则保留它，等待下一批数据中的55
                if (
                    self._rx_buffer
                    and self._rx_buffer[-1]
                    == self.response_protocol.START_BYTES[0]
                ):
                    self._rx_buffer[:] = self._rx_buffer[-1:]
                else:
                    self._rx_buffer.clear()

                return

            # 删除帧头前面的干扰字节
            if frame_start > 0:
                del self._rx_buffer[:frame_start]

            # 数据不足一帧时保留现有字节，等待下一批数据
            if (
                len(self._rx_buffer)
                < self.response_protocol.RESPONSE_FRAME_LEN
            ):
                return

            response_frame = bytes(
                self._rx_buffer[
                    :self.response_protocol.RESPONSE_FRAME_LEN
                ]
            )

            try:
                response_payload = self.response_protocol.decode_response(
                    response_frame
                )
            except (TypeError, ValueError) as error:
                self.last_error = error

                # 当前AA 55不是合法帧头，向后滑动一个字节重新搜索
                del self._rx_buffer[:1]
                continue

            # 在读取线程确认完整合法帧时生成从0开始的毫秒时间戳
            timestamp_ms = (time.perf_counter_ns() - self._start_time_ns) / 1_000_000.0

            # 将接收时间戳和传感器负载作为同一帧保存
            self._frame_buffer.append((timestamp_ms,response_payload))

            # 删除已经处理的完整回复帧，继续检查后面是否还有粘包
            del self._rx_buffer[
                :self.response_protocol.RESPONSE_FRAME_LEN
            ]

    def read_data(self):
        """返回(timestamp_ms, 188B传感器负载)，无数据时返回None"""

        with self._buffer_lock:
            if not self._frame_buffer:
                return None

            return self._frame_buffer.popleft()     # 从 deque 最左边取出一个元素，并同时从队列中删除它

    def close(self):

        # 先关线程，再关串口
        self._running.clear()                       # Event 设置为 False

        if (
            self._request_thread is not None
            and self._request_thread.is_alive()
        ):
            self._request_thread.join(timeout = 1)

        if (
            self._response_thread is not None
            and self._response_thread.is_alive()
        ):
            self._response_thread.join(timeout = 1)

        if (
            self.serial is not None
            and self.serial.is_open
        ):
            self.serial.close() 
