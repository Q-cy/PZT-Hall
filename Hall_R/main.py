"""
主函数
"""

from tactile_sensor import TactileSensor

def main():
    tactile_sensor = TactileSensor() # 实例化对象，会运行TactileSensor.__init__()

    try:                            # try 代码块内会进行异常捕获，捕获异常后执行 except，finally 无论如何都会执行
        while True:
            raw = tactile_sensor.read_data()

            if raw is not None:
                print(f"received_data: {len(raw)} bytes")

    except KeyboardInterrupt:
        print("\nstop received")

    finally:
        tactile_sensor.close()

if __name__ == "__main__":          # __name__ 为内置特殊变量，直接运行脚本时 __name__ == "__main__"，否则为模块名 main
    main()                          # 目的是为了同时可以(1)直接作为脚本运行(2)被其他模块调用但不自动执行

