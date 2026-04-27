#!/usr/bin/env python3
"""
空载转速-电压特性测试上位机程序

用法：
    python scripts/plot_speed_voltage.py

要求：
    pip install pyserial matplotlib

接线：
    STM32 PB6(TX) / PB7(RX) 通过 DAPLINK 虚拟串口连接到 PC
"""

import glob
import sys

import csv
import matplotlib.pyplot as plt
import serial
from collections import defaultdict


def find_serial_port():
    """自动查找 DAPLINK / CDC 串口设备"""
    patterns = ['/dev/ttyACM*', '/dev/ttyUSB*', '/dev/tty.usbmodem*']
    ports = []
    for p in patterns:
        ports.extend(glob.glob(p))
    if not ports:
        raise RuntimeError(
            "No serial port found. Please connect DAPLINK and check /dev/ttyACM*"
        )
    return ports[0]


def collect_data(ser):
    """发送开始命令并收集测试数据"""
    print("Sending start command 'S'...")
    ser.write(b'S')
    ser.flush()

    readings = []  # [(duty, rpm), ...]
    print("Collecting data (this takes a few minutes)...")

    while True:
        line = ser.readline().decode('ascii', errors='ignore').strip()
        if not line:
            continue
        if line == "DONE":
            print("Test finished by MCU.")
            break

        try:
            duty_str, rpm_str = line.split(',')
            duty = int(duty_str)
            rpm = float(rpm_str)
            readings.append((duty, rpm))
            print(f"  duty={duty:3d}%  rpm={rpm:7.1f}")
        except ValueError:
            print(f"  [skip invalid line: {line!r}]")

    return readings


def average_by_duty(readings):
    """按 duty 分组取平均（合并多轮往返数据）"""
    groups = defaultdict(list)
    for duty, rpm in readings:
        groups[duty].append(rpm)

    avg_data = []
    for duty in sorted(groups.keys()):
        rpms = groups[duty]
        avg_rpm = sum(rpms) / len(rpms)
        voltage = duty * 11.0 / 100.0  # 100% duty ≈ 11 V
        avg_data.append(
            {
                'duty': duty,
                'voltage_V': voltage,
                'rpm_avg': avg_rpm,
                'sample_count': len(rpms),
            }
        )
    return avg_data


def save_csv(avg_data, path='speed_voltage.csv'):
    """保存 CSV 文件"""
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(
            f, fieldnames=['duty_percent', 'voltage_V', 'rpm_avg', 'sample_count']
        )
        writer.writeheader()
        for row in avg_data:
            writer.writerow(
                {
                    'duty_percent': row['duty'],
                    'voltage_V': f"{row['voltage_V']:.2f}",
                    'rpm_avg': f"{row['rpm_avg']:.1f}",
                    'sample_count': row['sample_count'],
                }
            )
    print(f"Saved CSV to {path}")


def plot(avg_data, path='speed_voltage.png'):
    """绘制转速-电压曲线"""
    duties = [r['duty'] for r in avg_data]
    voltages = [r['voltage_V'] for r in avg_data]
    rpms = [r['rpm_avg'] for r in avg_data]

    fig, ax1 = plt.subplots(figsize=(10, 6))

    # 主曲线：电压 vs 转速
    ax1.plot(
        voltages, rpms, 'b-o', linewidth=2, markersize=6, label='Avg RPM (no-load)'
    )
    ax1.set_xlabel('Voltage (V)', fontsize=12)
    ax1.set_ylabel('Speed (RPM)', fontsize=12, color='b')
    ax1.tick_params(axis='y', labelcolor='b')
    ax1.grid(True, linestyle='--', alpha=0.6)

    # 上方标注 duty 百分比
    ax2 = ax1.twiny()
    ax2.set_xlim(ax1.get_xlim())
    tick_indices = list(range(0, len(voltages), max(1, len(voltages) // 10)))
    ax2.set_xticks([voltages[i] for i in tick_indices])
    ax2.set_xticklabels([f"{duties[i]}%" for i in tick_indices])
    ax2.set_xlabel('Duty (%)', fontsize=12)

    # 标题与图例
    plt.title('No-Load Motor Speed vs Voltage\n(Multiple round-trip averaged)', fontsize=14)
    ax1.legend(loc='lower right')
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    print(f"Saved plot to {path}")
    plt.show()


def main():
    port = find_serial_port()
    print(f"Serial port: {port}")

    # 超时设长一些，整个测试可能需要 3~5 分钟
    with serial.Serial(port, 115200, timeout=600) as ser:
        readings = collect_data(ser)

    if not readings:
        print("No data received. Exiting.")
        sys.exit(1)

    avg_data = average_by_duty(readings)

    print("\n--- Averaged Results ---")
    for row in avg_data:
        print(
            f"  duty={row['duty']:3d}%  "
            f"voltage={row['voltage_V']:4.2f}V  "
            f"rpm_avg={row['rpm_avg']:7.1f}  "
            f"(n={row['sample_count']})"
        )

    save_csv(avg_data)
    plot(avg_data)


if __name__ == '__main__':
    main()
