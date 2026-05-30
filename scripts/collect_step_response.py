#!/usr/bin/env python3
"""
空载阶跃响应实验上位机 —— 数据收集程序

用法：
    python scripts/collect_step_response.py [output_dir]

要求：
    pip install pyserial

接线：
    STM32 PB6(TX) / PB7(RX) 通过 DAPLINK 虚拟串口连接到 PC

流程：
    1. 连接串口，发送 'S' 命令启动 MCU 实验程序
    2. 接收 META 元信息
    3. 逐轮接收数据并保存为结构化文件
    4. 实验结束后保存原始日志和解析后的 CSV
"""

import csv
import glob
import json
import os
import sys
from collections import defaultdict
from datetime import datetime

import serial


def find_serial_port():
    """自动查找 DAPLINK / CDC 串口设备"""
    patterns = ["/dev/ttyACM*", "/dev/ttyUSB*", "/dev/tty.usbmodem*"]
    ports = []
    for p in patterns:
        ports.extend(glob.glob(p))
    if not ports:
        raise RuntimeError(
            "No serial port found. Please connect DAPLINK and check /dev/ttyACM*"
        )
    return ports[0]


def parse_meta(line: str) -> dict:
    """解析 META 行: META,sample_period_ms=50,hold_ms=3000,..."""
    meta = {}
    parts = line.strip().split(",")
    # 找到 duty_levels= 的位置，之后全是 duty 数值
    duty_start = None
    for i, part in enumerate(parts[1:], start=1):
        if part.startswith("duty_levels="):
            duty_start = i
            # duty_levels= 后面可能紧跟第一个 duty 值（如 duty_levels=40）
            rest = part[len("duty_levels="):]
            if rest:
                meta["duty_levels"] = [int(x) for x in rest.split(",") if x]
            else:
                meta["duty_levels"] = []
            # 收集后续所有纯数字片段作为 duty_levels
            for j in range(i + 1, len(parts)):
                if parts[j].strip().isdigit():
                    meta["duty_levels"].append(int(parts[j]))
            break
        elif "=" in part:
            k, v = part.split("=", 1)
            try:
                meta[k] = int(v)
            except ValueError:
                meta[k] = v
    return meta


def collect_data(ser: serial.Serial):
    """发送开始命令并收集所有实验数据"""
    print("Sending start command 'S'...")
    ser.write(b"S")
    ser.flush()

    meta = None
    raw_lines = []
    runs = []  # 列表，每个元素是一个运行的数据
    current_run = None

    print("Collecting data (this takes a few minutes)...")

    while True:
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
        except UnicodeDecodeError:
            continue

        if not line:
            continue

        raw_lines.append(line)

        # 元信息行
        if line.startswith("META,"):
            meta = parse_meta(line)
            print(f"  Meta: {meta}")
            continue

        # 单次运行开始
        if line.startswith("START,"):
            # START,duty=40,rep=1
            parts = line.split(",")
            duty = int(parts[1].split("=")[1])
            rep = int(parts[2].split("=")[1])
            current_run = {
                "duty": duty,
                "rep": rep,
                "data": [],  # [(time_ms, rpm, duty), ...]
            }
            print(f"  -> Start duty={duty}% rep={rep}")
            continue

        # 单次运行结束
        if line.startswith("END,"):
            if current_run is not None:
                runs.append(current_run)
                current_run = None
            continue

        # 全部完成
        if line == "ALL_DONE":
            print("  All tests finished by MCU.")
            break

        # 数据行: time_ms,rpm,duty
        try:
            t_str, rpm_str, d_str = line.split(",")
            t = int(t_str)
            rpm = float(rpm_str)
            d = int(d_str)
            if current_run is not None:
                current_run["data"].append((t, rpm, d))
        except ValueError:
            print(f"  [skip invalid line: {line!r}]")

    return meta, runs, raw_lines


def save_csv(runs: list, output_dir: str):
    """保存所有运行数据到一个 CSV 文件"""
    csv_path = os.path.join(output_dir, "step_response_data.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["duty", "rep", "time_ms", "rpm"])
        for run in runs:
            duty = run["duty"]
            rep = run["rep"]
            for t, rpm, _ in run["data"]:
                writer.writerow([duty, rep, t, rpm])
    print(f"Saved CSV to {csv_path}")
    return csv_path


def save_json(runs: list, meta: dict, output_dir: str):
    """保存结构化 JSON，便于后续分析"""
    json_path = os.path.join(output_dir, "step_response_data.json")
    # 按 duty 分组
    grouped = defaultdict(lambda: {"duty": 0, "reps": []})
    for run in runs:
        d = run["duty"]
        if grouped[d]["duty"] == 0:
            grouped[d]["duty"] = d
        grouped[d]["reps"].append(
            {
                "rep": run["rep"],
                "time_ms": [x[0] for x in run["data"]],
                "rpm": [x[1] for x in run["data"]],
            }
        )

    payload = {
        "meta": meta,
        "recorded_at": datetime.now().isoformat(),
        "runs": [
            {"duty": v["duty"], "repetitions": v["reps"]} for v in grouped.values()
        ],
    }

    with open(json_path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"Saved JSON to {json_path}")
    return json_path


def save_raw(raw_lines: list, output_dir: str):
    """保存原始串口日志"""
    raw_path = os.path.join(output_dir, "step_response_raw.log")
    with open(raw_path, "w") as f:
        for line in raw_lines:
            f.write(line + "\n")
    print(f"Saved raw log to {raw_path}")


def main():
    output_dir = sys.argv[1] if len(sys.argv) > 1 else "step_response"
    os.makedirs(output_dir, exist_ok=True)
    print(f"Output directory: {output_dir}")

    port = find_serial_port()
    print(f"Serial port: {port}")

    # 超时设长，整个实验可能需要 5~10 分钟
    with serial.Serial(port, 115200, timeout=600) as ser:
        meta, runs, raw_lines = collect_data(ser)

    if not runs:
        print("No data received. Exiting.")
        sys.exit(1)

    save_raw(raw_lines, output_dir)
    csv_path = save_csv(runs, output_dir)
    json_path = save_json(runs, meta, output_dir)

    print("\n=== Collection Summary ===")
    print(f"  Total runs: {len(runs)}")
    if meta:
        print(f"  Duty levels: {meta.get('duty_levels', 'N/A')}")
        print(f"  Repetitions per level: {meta.get('reps', 'N/A')}")
        print(f"  Sample period: {meta.get('sample_period_ms', 'N/A')} ms")
    print(f"\nNext step: python scripts/analyze_step_response.py {json_path}")


if __name__ == "__main__":
    main()
