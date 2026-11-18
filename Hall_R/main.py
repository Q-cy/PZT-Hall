"""
主函数
"""

import time

from tactile_sensor import TactileSensor


def main():
    tactile_sensor = TactileSensor() # 实例化对象，会运行TactileSensor.__init__()

    try:                            # try 代码块内会进行异常捕获，捕获异常后执行 except，finally 无论如何都会执行
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

            print(
                "raw payload:",
                response_payload.hex(" "),
            )

            print(
                "piezoresistance:",
                piezoresistance_data,
            )

            for hall_index, hall_raw in enumerate(
                hall_data,
                start = 1,
            ):
                print(
                    f"hall {hall_index}:",
                    hall_raw.hex(" "),
                )

    except KeyboardInterrupt:
        print("\nstop received")

    finally:
        tactile_sensor.close()


if __name__ == "__main__":          # __name__ 为内置特殊变量，直接运行脚本时 __name__ == "__main__"，否则为模块名 main
    main()                          # 目的是为了同时可以(1)直接作为脚本运行(2)被其他模块调用但不自动执行
