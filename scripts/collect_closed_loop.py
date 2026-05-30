#!/usr/bin/env python3
"""
闭环前馈-反馈阶跃响应 + 阶跃扰动实验上位机 —— 数据收集程序

用法：
    python scripts/collect_closed_loop.py [output_dir]

要求：
    pip install pyserial

接线：
    STM32 PB6(TX) / PB7(RX) 通过 DAPLINK 虚拟串口连接到 PC

流程：
    1. 连接串口，发送 'C' 命令启动 MCU 实验程序
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
    """解析 META 行"""
    meta = {}
    parts = line.strip().split(",")
    duty_start = None
    for i, part in enumerate(parts[1:], start=1):
        if part.startswith("step_setpoints="):
            rest = part[len("step_setpoints="):]
            meta["step_setpoints"] = [float(x) for x in rest.split(",") if x] if rest else []
            for j in range(i + 1, len(parts)):
                if parts[j].strip().replace(".", "").replace("-", "").isdigit():
                    meta["step_setpoints"].append(float(parts[j]))
                else:
                    break
            break
        elif "=" in part:
            k, v = part.split("=", 1)
            try:
                meta[k] = int(v)
            except ValueError:
                try:
                    meta[k] = float(v)
                except ValueError:
                    meta[k] = v
    return meta


def collect_data(ser: serial.Serial):
    """发送开始命令并收集所有实验数据"""
    print("Sending start command 'C'...")
    ser.write(b"C")
    ser.flush()

    meta = None
    raw_lines = []
    runs = []
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

        if line.startswith("META,"):
            meta = parse_meta(line)
            print(f"  Meta: {meta}")
            continue

        if line.startswith("START,"):
            parts = line.split(",")
            d = {}
            for p in parts[1:]:
                if "=" in p:
                    k, v = p.split("=", 1)
                    d[k] = v
            current_run = {
                "typ": d.get("typ", "UNKNOWN"),
                "sp": float(d.get("sp", 0)),
                "disturb": float(d.get("disturb", 0)),
                "rep": int(d.get("rep", 0)),
                "data": [],
            }
            print(f"  -> Start {current_run['typ']} sp={current_run['sp']} rep={current_run['rep']}")
            continue

        if line.startswith("END,"):
            if current_run is not None:
                runs.append(current_run)
                current_run = None
            continue

        if line == "ALL_DONE":
            print("  All tests finished by MCU.")
            break

        # 数据行: time_ms,sp_rpm,pv_rpm,duty_total,duty_ff,duty_fb,disturbance
        try:
            t_str, sp_str, pv_str, dt_str, ff_str, fb_str, dist_str = line.split(",")
            t = int(t_str)
            sp = float(sp_str)
            pv = float(pv_str)
            dt = float(dt_str)
            ff = float(ff_str)
            fb = float(fb_str)
            dist = float(dist_str)
            if current_run is not None:
                current_run["data"].append((t, sp, pv, dt, ff, fb, dist))
        except ValueError:
            print(f"  [skip invalid line: {line!r}]")

    return meta, runs, raw_lines


def save_csv(runs: list, output_dir: str):
    csv_path = os.path.join(output_dir, "closed_loop_data.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["typ", "sp", "disturb", "rep", "time_ms", "sp_rpm", "pv_rpm", "duty_total", "duty_ff", "duty_fb", "disturbance"])
        for run in runs:
            for t, sp, pv, dt, ff, fb, dist in run["data"]:
                writer.writerow([run["typ"], run["sp"], run["disturb"], run["rep"], t, sp, pv, dt, ff, fb, dist])
    print(f"Saved CSV to {csv_path}")
    return csv_path


def save_json(runs: list, meta: dict, output_dir: str):
    json_path = os.path.join(output_dir, "closed_loop_data.json")
    grouped = defaultdict(lambda: {"typ": "", "sp": 0.0, "disturb": 0.0, "reps": []})
    for run in runs:
        key = (run["typ"], run["sp"], run["disturb"])
        if grouped[key]["typ"] == "":
            grouped[key]["typ"] = run["typ"]
            grouped[key]["sp"] = run["sp"]
            grouped[key]["disturb"] = run["disturb"]
        grouped[key]["reps"].append({
            "rep": run["rep"],
            "time_ms": [x[0] for x in run["data"]],
            "sp_rpm": [x[1] for x in run["data"]],
            "pv_rpm": [x[2] for x in run["data"]],
            "duty_total": [x[3] for x in run["data"]],
            "duty_ff": [x[4] for x in run["data"]],
            "duty_fb": [x[5] for x in run["data"]],
            "disturbance": [x[6] for x in run["data"]],
        })

    payload = {
        "meta": meta,
        "recorded_at": datetime.now().isoformat(),
        "runs": [
            {"typ": v["typ"], "sp": v["sp"], "disturb": v["disturb"], "repetitions": v["reps"]}
            for v in grouped.values()
        ],
    }

    with open(json_path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"Saved JSON to {json_path}")
    return json_path


def save_raw(raw_lines: list, output_dir: str):
    raw_path = os.path.join(output_dir, "closed_loop_raw.log")
    with open(raw_path, "w") as f:
        for line in raw_lines:
            f.write(line + "\n")
    print(f"Saved raw log to {raw_path}")


def main():
    output_dir = sys.argv[1] if len(sys.argv) > 1 else "closed_loop_response"
    os.makedirs(output_dir, exist_ok=True)
    print(f"Output directory: {output_dir}")

    port = find_serial_port()
    print(f"Serial port: {port}")

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
        print(f"  Step setpoints: {meta.get('step_setpoints', 'N/A')}")
        print(f"  Repetitions: {meta.get('reps', 'N/A')}")
        print(f"  Sample period: {meta.get('sample_period_ms', 'N/A')} ms")
        print(f"  Control period: {meta.get('control_period_ms', 'N/A')} ms")
    print(f"\nNext step: python scripts/analyze_closed_loop.py {json_path}")


if __name__ == "__main__":
    main()
