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
TACTILE_SENSOR_SERIAL_TIMEOUT = 0.01
TACTILE_SENSOR_SEND_INTERVAL = 0.005
TACTILE_SENSOR_READ_INTERVAL = 0.001


# ==================== Sensor Config ====================

PIEZORESISTANCE_ROWS = 13
PIEZORESISTANCE_COLS = 6
HALL_SENSOR_COUNT = 4
HALL_SENSOR_LEN = 8


# ==================== Protocol Config ====================

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
    """CRC-8-ITU：poly=0x07, init=0x00, final XOR=0x55"""
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
    """串口读操作请求帧"""

    START_BYTES = b"\x55\xAA"

    REQUEST_CONFIG_LEN = 9
    REQUEST_DATA_LEN_BYTES = struct.pack("<H", REQUEST_CONFIG_LEN)

    DEVICE_ADDRESS = 0x34
    RESERVED = 0x00
    FUNCTION_CODE = 0xFB

    START_ADDRESS = 0x00001C00
    START_ADDRESS_BYTES = struct.pack("<I", START_ADDRESS)

    READ_DATA_LEN_BYTES = struct.pack("<H", RESPONSE_PAYLOAD_LEN)

    def __init__(self):
        self.request_frame = self.build_request_frame()

    def build_request_frame(self):
        """根据Sensor Config自动生成请求帧"""

        request_frame_withoutcrc = (
            self.START_BYTES
            + self.REQUEST_DATA_LEN_BYTES
            + bytes([
                self.DEVICE_ADDRESS,
                self.RESERVED,
                self.FUNCTION_CODE,
            ])
            + self.START_ADDRESS_BYTES
            + self.READ_DATA_LEN_BYTES
        )

        request_frame_crc = crc8_itu(
            request_frame_withoutcrc
        )

        return (
            request_frame_withoutcrc
            + bytes([request_frame_crc])
        )


# ==================== Response Protocol ====================

class RESPONSE_PROTOCOL:
    """串口读操作回复帧"""

    START_BYTES_LEN = 2
    START_BYTES = b"\xAA\x55"
    RESPONSE_DATA_LEN_FIELD_LEN = 2

    DEVICE_ADDRESS_LEN = 1
    DEVICE_ADDRESS = 0x34
    RESERVED_LEN = 1
    RESERVED = 0x00
    FUNCTION_CODE_LEN = 1
    FUNCTION_CODE = 0xFB

    START_ADDRESS_LEN = 4
    START_ADDRESS = 0x00001C00
    START_ADDRESS_BYTES = struct.pack("<I", START_ADDRESS)

    RESPONSE_BYTES_NUM_LEN = 2
    RESPONSE_BYTES_NUM_BYTES = struct.pack("<H", RESPONSE_PAYLOAD_LEN)
    STATE_LEN = 1
    STATE_SUCCESS = 0x00
    RESPONSE_CRC_LEN = 1

    RESPONSE_FRAME_HEADER_LEN = START_BYTES_LEN + RESPONSE_DATA_LEN_FIELD_LEN

    RESPONSE_CONFIG_LEN = (
        DEVICE_ADDRESS_LEN
        + RESERVED_LEN
        + FUNCTION_CODE_LEN
        + START_ADDRESS_LEN
        + RESPONSE_BYTES_NUM_LEN
        + STATE_LEN
    )

    RESPONSE_DATA_LEN = RESPONSE_CONFIG_LEN + RESPONSE_PAYLOAD_LEN
    RESPONSE_DATA_LEN_BYTES = struct.pack("<H", RESPONSE_DATA_LEN)

    RESPONSE_FRAME_LEN = (
        RESPONSE_FRAME_HEADER_LEN
        + RESPONSE_DATA_LEN
        + RESPONSE_CRC_LEN
    )

    PAYLOAD_START = RESPONSE_FRAME_HEADER_LEN + RESPONSE_CONFIG_LEN
    PAYLOAD_END = PAYLOAD_START + RESPONSE_PAYLOAD_LEN
    CRC_START = PAYLOAD_END

    def decode_response(self, response_frame):
        """校验回复帧，并提取传感器原始负载"""

        if not isinstance(response_frame, bytes):
            raise TypeError("response_frame must be bytes")

        if len(response_frame) != self.RESPONSE_FRAME_LEN:
            raise ValueError(
                f"response len error: received {len(response_frame)} bytes, "
                f"should be {self.RESPONSE_FRAME_LEN} bytes"
            )

        if response_frame[0:2] != self.START_BYTES:
            raise ValueError("start bytes error")

        if response_frame[2:4] != self.RESPONSE_DATA_LEN_BYTES:
            raise ValueError("response data len error")

        if response_frame[4] != self.DEVICE_ADDRESS:
            raise ValueError("device address error")

        if response_frame[5] != self.RESERVED:
            raise ValueError("reserved byte error")

        if response_frame[6] != self.FUNCTION_CODE:
            raise ValueError("function code error")

        if response_frame[7:11] != self.START_ADDRESS_BYTES:
            raise ValueError("start address error")

        if response_frame[11:13] != self.RESPONSE_BYTES_NUM_BYTES:
            raise ValueError("response data num error")

        if response_frame[13] != self.STATE_SUCCESS:
            raise ValueError(
                f"state error: 0x{response_frame[13]:02X}"
            )

        response_crc = response_frame[self.CRC_START]
        calculated_crc = crc8_itu(response_frame[:self.CRC_START])

        if response_crc != calculated_crc:
            raise ValueError(
                f"CRC error: received 0x{response_crc:02X}, "
                f"calculated 0x{calculated_crc:02X}"
            )

        return response_frame[
            self.PAYLOAD_START:self.PAYLOAD_END
        ]

    def decode_payload(self, response_payload):
        """将传感器原始负载切分为压阻数据和Hall原始数据"""

        if not isinstance(response_payload, bytes):
            raise TypeError("response_payload must be bytes")

        if len(response_payload) != RESPONSE_PAYLOAD_LEN:
            raise ValueError(
                f"payload len error: received {len(response_payload)} bytes, "
                f"should be {RESPONSE_PAYLOAD_LEN} bytes"
            )

        piezoresistance_payload = response_payload[
            :PIEZORESISTANCE_PAYLOAD_LEN
        ]

        piezoresistance_data = list(
            struct.unpack(
                f"<{PIEZORESISTANCE_CELL_COUNT}H",
                piezoresistance_payload,
            )
        )

        hall_payload = response_payload[
            PIEZORESISTANCE_PAYLOAD_LEN:
        ]

        hall_data = []

        for hall_index in range(HALL_SENSOR_COUNT):
            hall_start = hall_index * HALL_SENSOR_LEN
            hall_end = hall_start + HALL_SENSOR_LEN

            hall_data.append(
                hall_payload[hall_start:hall_end]
            )

        return piezoresistance_data, hall_data


# ==================== UART Control ====================

class TactileSensor:
    MAX_RX_BUFFER_LEN = max(
        4096,
        RESPONSE_PROTOCOL.RESPONSE_FRAME_LEN * 16,
    )
    FRAME_BUFFER_MAXLEN = 256

    def __init__(
        self,
        port = TACTILE_SENSOR_PORT,
        baudrate = TACTILE_SENSOR_DATA_BAUDRATE,
    ):
        self.port = port
        self.baudrate = baudrate
        self.ser = None

        self.request_protocol = REQUEST_PROTOCOL()
        self.response_protocol = RESPONSE_PROTOCOL()

        self._rx_buffer = bytearray()
        self._frame_buffer = deque(maxlen = self.FRAME_BUFFER_MAXLEN)
        self._buffer_lock = threading.Lock()
        self._running = threading.Event()

        self._send_thread = None
        self._read_thread = None
        self._start_time_ns = None
        self.last_error = None

        self.open_port()
        self.start()

    def open_port(self):
        """打开触觉传感器串口"""

        self.ser = serial.Serial(
            port = self.port,
            baudrate = self.baudrate,
            timeout = TACTILE_SENSOR_SERIAL_TIMEOUT,
        )

        time.sleep(0.1)
        self.ser.reset_input_buffer()

    def start(self):
        """启动发送线程和读取线程"""

        if self._running.is_set():
            return

        self._start_time_ns = time.perf_counter_ns()
        self._running.set()

        self._send_thread = threading.Thread(
            target = self._send_loop,
            daemon = True,
        )

        self._read_thread = threading.Thread(
            target = self._read_loop,
            daemon = True,
        )

        self._send_thread.start()
        self._read_thread.start()

    def _send_loop(self):
        """每5ms发送一次读取请求"""

        while self._running.is_set():
            try:
                self.ser.write(
                    self.request_protocol.request_frame
                )
                self.ser.flush()

            except (serial.SerialException, OSError) as error:
                self.last_error = error

            time.sleep(TACTILE_SENSOR_SEND_INTERVAL)

    def _read_loop(self):
        """持续读取串口数据，并解析完整回复帧"""

        while self._running.is_set():
            try:
                waiting_bytes = self.ser.in_waiting

                if waiting_bytes <= 0:
                    time.sleep(TACTILE_SENSOR_READ_INTERVAL)
                    continue

                received_data = self.ser.read(waiting_bytes)

                if received_data:
                    self._append_received_data(received_data)

            except (serial.SerialException, OSError) as error:
                self.last_error = error
                time.sleep(TACTILE_SENSOR_READ_INTERVAL)

    def _append_received_data(self, received_data):
        """将串口数据加入缓存，并解析其中的完整帧"""

        with self._buffer_lock:
            self._rx_buffer.extend(received_data)

            if len(self._rx_buffer) > self.MAX_RX_BUFFER_LEN:
                del self._rx_buffer[:-self.MAX_RX_BUFFER_LEN]

            self._parse_response_buffer()

    def _parse_response_buffer(self):
        """从接收缓存中查找并取出完整回复帧"""

        while True:
            frame_start = self._rx_buffer.find(
                self.response_protocol.START_BYTES
            )

            if frame_start < 0:
                if (
                    self._rx_buffer
                    and self._rx_buffer[-1]
                    == self.response_protocol.START_BYTES[0]
                ):
                    self._rx_buffer[:] = self._rx_buffer[-1:]
                else:
                    self._rx_buffer.clear()

                return

            if frame_start > 0:
                del self._rx_buffer[:frame_start]

            if len(self._rx_buffer) < self.response_protocol.RESPONSE_FRAME_LEN:
                return

            response_frame = bytes(
                self._rx_buffer[:self.response_protocol.RESPONSE_FRAME_LEN]
            )

            try:
                response_payload = self.response_protocol.decode_response(
                    response_frame
                )

            except (TypeError, ValueError) as error:
                self.last_error = error
                del self._rx_buffer[:1]
                continue

            # 在读取线程确认完整有效帧时生成从0开始的毫秒时间戳
            timestamp_ms = (
                time.perf_counter_ns()
                - self._start_time_ns
            ) / 1_000_000.0

            self._frame_buffer.append((
                timestamp_ms,
                response_frame,
                response_payload,
            ))

            del self._rx_buffer[
                :self.response_protocol.RESPONSE_FRAME_LEN
            ]

    def _pop_frame_packet(self):
        """从缓存中取出最早的一帧"""

        with self._buffer_lock:
            if not self._frame_buffer:
                return None

            return self._frame_buffer.popleft()

    def read_raw_frame(self):
        """返回(timestamp_ms, 203B完整回复帧)"""

        frame_packet = self._pop_frame_packet()

        if frame_packet is None:
            return None

        timestamp_ms, response_frame, _ = frame_packet
        return timestamp_ms, response_frame

    def read_data(self):
        """返回(timestamp_ms, 188B传感器原始负载)"""

        frame_packet = self._pop_frame_packet()

        if frame_packet is None:
            return None

        timestamp_ms, _, response_payload = frame_packet
        return timestamp_ms, response_payload

    def decode(self, response_payload):
        """返回(压阻数据列表, Hall原始数据列表)"""

        return self.response_protocol.decode_payload(
            response_payload
        )

    def close(self):
        """停止线程并关闭触觉传感器串口"""

        self._running.clear()

        if self._send_thread is not None and self._send_thread.is_alive():
            self._send_thread.join(timeout = 1)

        if self._read_thread is not None and self._read_thread.is_alive():
            self._read_thread.join(timeout = 1)

        if self.ser is not None and self.ser.is_open:
            self.ser.close()


# ==================== Main ====================

def main():
    tactile_sensor = TactileSensor()

    try:
        print(
            "request frame:",
            tactile_sensor.request_protocol.request_frame.hex(" "),
        )

        while True:
            sensor_packet = tactile_sensor.read_data()

            if sensor_packet is None:
                time.sleep(0.001)
                continue

            timestamp_ms, response_payload = sensor_packet
            piezoresistance_data, hall_data = tactile_sensor.decode(
                response_payload
            )

            print(
                f"timestamp: {timestamp_ms:.3f} ms | "
                f"payload: {len(response_payload)} bytes"
            )
            print("raw payload:", response_payload.hex(" "))
            print("piezoresistance:", piezoresistance_data)

            for hall_index, hall_sensor_data in enumerate(
                hall_data,
                start = 1,
            ):
                print(
                    f"hall {hall_index}:",
                    hall_sensor_data.hex(" "),
                )

    except KeyboardInterrupt:
        print("\nstop received")

    finally:
        tactile_sensor.close()


if __name__ == "__main__":
    main()
