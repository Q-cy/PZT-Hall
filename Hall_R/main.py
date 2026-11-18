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
    frame_count = 0
    print_window_start_ms = None
    print_window_frame_count = 0

    try:                            # try 代码块内会进行异常捕获，捕获异常后执行 except，finally 无论如何都会执行
        while True:
            sensor_packet = tactile_sensor.read_data()

            if sensor_packet is None:
                time.sleep(0.001)
                continue

            timestamp_ms, response_payload = sensor_packet
            frame_count += 1
            print_window_frame_count += 1

            if print_window_start_ms is None:
                print_window_start_ms = timestamp_ms

            print_window_elapsed_ms = (
                timestamp_ms - print_window_start_ms
            )

            # 主循环继续逐帧取数据，但终端每约1秒只打印一次
            if print_window_elapsed_ms < 1000.0:
                continue

            frame_rate = (
                print_window_frame_count
                * 1000.0
                / print_window_elapsed_ms
            )

            piezoresistance_data, hall_data = tactile_sensor.decode(
                response_payload
            )

            print(
                f"frame: {frame_count} | "
                f"timestamp: {timestamp_ms:.3f} ms | "
                f"fps: {frame_rate:.1f} | "
                f"payload: {len(response_payload)} bytes"
            )

            print("piezoresistance:")
            print(
                format_piezoresistance(
                    piezoresistance_data
                )
            )

            for hall_index, hall_raw in enumerate(
                hall_data,
                start = 1,
            ):
                print(
                    f"hall {hall_index}:",
                    hall_raw.hex(" "),
                )

            print_window_start_ms = timestamp_ms
            print_window_frame_count = 0

    except KeyboardInterrupt:
        print("\nstop received")

    finally:
        tactile_sensor.close()


if __name__ == "__main__":          # __name__ 为内置特殊变量，直接运行脚本时 __name__ == "__main__"，否则为模块名 main
    main()                          # 目的是为了同时可以(1)直接作为脚本运行(2)被其他模块调用但不自动执行
