"""
主函数
"""

import time

from tactile_sensor import (
    PIEZORESISTANCE_COLS,
    PIEZORESISTANCE_ROWS,
    TactileSensor,
)


PIEZORESISTANCE_VALUE_WIDTH = 5


def format_piezoresistance(piezoresistance_data):
    """按照Sensor Config的行列数格式化固定列宽压力矩阵"""

    expected_count = (
        PIEZORESISTANCE_ROWS
        * PIEZORESISTANCE_COLS
    )

    if len(piezoresistance_data) != expected_count:
        raise ValueError(
            f"piezoresistance count error: "
            f"received {len(piezoresistance_data)}, "
            f"should be {expected_count}"
        )

    matrix_lines = []

    for row_index in range(PIEZORESISTANCE_ROWS):
        row_start = row_index * PIEZORESISTANCE_COLS
        row_end = row_start + PIEZORESISTANCE_COLS
        row_data = piezoresistance_data[row_start:row_end]

        matrix_lines.append(
            " ".join(
                f"{value:{PIEZORESISTANCE_VALUE_WIDTH}d}"
                for value in row_data
            )
        )

    return "\n".join(matrix_lines)


def main():
    tactile_sensor = TactileSensor() # 实例化对象，会运行TactileSensor.__init__()
    latest_timestamp_ms = None
    latest_response_payload = None

    print_window_start_time = time.monotonic()
    previous_status = tactile_sensor.get_status()
    active_problem = None

    print(
        f"serial opened: {tactile_sensor.port} @ "
        f"{tactile_sensor.baudrate} baud"
    )
    print(
        "request frame:",
        tactile_sensor.request_protocol.request_frame.hex(" ")
    )

    try:                            # try 代码块内会进行异常捕获，捕获异常后执行 except，finally 无论如何都会执行
        while True:
            sensor_packet = tactile_sensor.read_data()

            if sensor_packet is not None:
                (
                    latest_timestamp_ms,
                    latest_response_payload,
                ) = sensor_packet

            current_time = time.monotonic()
            print_window_elapsed_s = (
                current_time - print_window_start_time
            )

            # 正常数据每秒打印一次；同一种故障只在首次出现时提示
            if print_window_elapsed_s < 1.0:
                if sensor_packet is None:
                    time.sleep(0.001)
                continue

            status = tactile_sensor.get_status()
            valid_frame_count = (
                status["valid_frame_count"]
                - previous_status["valid_frame_count"]
            )
            invalid_frame_count = (
                status["invalid_frame_count"]
                - previous_status["invalid_frame_count"]
            )
            received_byte_count = (
                status["received_byte_count"]
                - previous_status["received_byte_count"]
            )
            frame_rate = valid_frame_count / print_window_elapsed_s

            if status["serial_error"] is not None:
                print("serial error:", status["serial_error"])
                break

            if (
                not status["request_thread_alive"]
                or not status["response_thread_alive"]
            ):
                problem = "acquisition_thread_stopped"

                if problem != active_problem:
                    print(
                        "acquisition thread stopped: "
                        f"TX={status['request_thread_alive']}, "
                        f"RX={status['response_thread_alive']}"
                    )

                active_problem = problem
                previous_status = status
                print_window_start_time = current_time
                continue

            if valid_frame_count <= 0:
                if received_byte_count <= 0:
                    problem = (
                        "no_serial_bytes",
                        "no bytes received from serial port"
                    )
                else:
                    problem = (
                        "no_valid_frame",
                        "bytes were received, but no complete "
                        "valid frame was decoded"
                    )

                if problem[0] != active_problem:
                    print("diagnosis:", problem[1])

                    if (
                        invalid_frame_count > 0
                        and status["last_error"] is not None
                    ):
                        print(
                            "last protocol error:",
                            status["last_error"]
                        )

                active_problem = problem[0]

                previous_status = status
                print_window_start_time = current_time
                continue

            if invalid_frame_count > 0:
                problem = "invalid_frame"

                if problem != active_problem:
                    print(
                        "protocol warning:",
                        f"{invalid_frame_count} invalid frame(s);",
                        status["last_error"]
                    )

                active_problem = problem
            else:
                active_problem = None

            piezoresistance_data, hall_data = tactile_sensor.decode(
                latest_response_payload
            )

            print(
                f"frame: {status['valid_frame_count']} | "
                f"timestamp: {latest_timestamp_ms:.3f} ms | "
                f"fps: {frame_rate:.1f} | "
                f"payload: {len(latest_response_payload)} bytes | "
                f"queued: {status['queued_frame_count']}"
            )

            print("piezoresistance:")
            print(
                format_piezoresistance(
                    piezoresistance_data
                )
            )

            print(
                "Hall  Temp(C)      Bx(G)      By(G)      Bz(G)     |B|(G)"
            )

            for hall_index, hall_sensor in enumerate(
                hall_data,
                start = 1,
            ):
                print(
                    f"{hall_index:4d} "
                    f"{hall_sensor['temperature_c']:8.2f} "
                    f"{hall_sensor['magnetic_x_gauss']:10.3f} "
                    f"{hall_sensor['magnetic_y_gauss']:10.3f} "
                    f"{hall_sensor['magnetic_z_gauss']:10.3f} "
                    f"{hall_sensor['magnetic_magnitude_gauss']:10.3f}"
                )

            previous_status = status
            print_window_start_time = current_time

    except KeyboardInterrupt:
        print("\nstop received")

    finally:
        tactile_sensor.close()


if __name__ == "__main__":          # __name__ 为内置特殊变量，直接运行脚本时 __name__ == "__main__"，否则为模块名 main
    main()                          # 目的是为了同时可以(1)直接作为脚本运行(2)被其他模块调用但不自动执行
