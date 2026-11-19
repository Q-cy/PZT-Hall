---
name: CSV 输出路径改为 Hall_R/data 文件夹
overview: 将 main.py 中 CSV 文件的输出位置从当前工作目录改为项目 data 文件夹（Hall_R/data），路径基于脚本所在位置解析，不依赖运行时 CWD。
todos:
  - id: update-csv-output-dir
    content: 修改 main.py：添加 import os 与 DATA_DIR 常量，CSV 路径改为 Hall_R/data 并 makedirs 容错创建
    status: completed
  - id: verify-data-dir
    content: 用 ast.parse 做语法检查，并只读验证 DATA_DIR 解析结果指向 Hall_R/data
    status: completed
    dependencies:
      - update-csv-output-dir
---

## 产品概述
触觉传感器采集程序已完成 CSV 数据记录功能（每帧一行、99 列、时间戳文件名）。用户移动了文件位置并要求调整 CSV 输出路径。

## 需求内容
- 文件位置已调整：`main.py` 与 `tactile_sensor.py` 移至 `Hall_R/code/`，并新建了空的 `Hall_R/data/` 文件夹
- CSV 文件保存到 `Hall_R/data/` 文件夹（即 `tactile_data_YYYYMMDD_HHMMSS.csv` 输出到 data 目录）
- 路径解析基于脚本所在位置（`__file__`），保证从任何工作目录运行脚本时 CSV 都输出到 `Hall_R/data/`，不依赖运行时 CWD
- 其余功能不变：每帧一行、写表头、每秒 flush、finally 关闭文件、保留终端每秒打印与故障诊断

## 技术方案
- 仅修改 `/home/qcy/Project/code/Tangential/PZT_Hall/Hall_R/code/main.py`；`tactile_sensor.py` 不改（CSV 输出路径属于 main 的运行配置）
- 顶部添加 `import os`
- 模块级定义 `DATA_DIR` 常量，基于脚本位置向上找一级 `data` 目录：
  ```python
  DATA_DIR = os.path.normpath(
      os.path.join(
          os.path.dirname(os.path.abspath(__file__)),
          "..",
          "data",
      )
  )
  ```
- `main()` 中创建 CSV 前调用 `os.makedirs(DATA_DIR, exist_ok=True)` 容错创建目录，并用 `os.path.join(DATA_DIR, time.strftime("tactile_data_%Y%m%d_%H%M%S.csv"))` 生成完整路径
- 其余逻辑（`build_csv_header`/`make_csv_row` 调用、writerow、flush、close、启动打印 `csv file: {csv_filename}`）保持不变
- 验证：`ast.parse` 语法检查；只读模拟 `__file__` 路径计算，确认 `DATA_DIR` 解析为 `/home/qcy/Project/code/Tangential/PZT_Hall/Hall_R/data`（不实际创建文件）
