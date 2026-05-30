#!/usr/bin/env python3
"""
闭环前馈-反馈控制器仿真验证脚本

基于已辨识空载电机模型：
    Gp(s) = K / (τ·s + 1)   [rpm/V]
    K = 23.4 rpm/V, τ = 0.017 s

仿真离散系统（Ts = 50 ms 控制周期，10 ms 采样周期）：
    - 电机模型用 ZOH 离散化
    - PI 控制器与固件实现一致
    - 加入测速量化噪声与随机扰动

输出与固件兼容的 JSON 格式，供 analyze_closed_loop.py 分析。

用法：
    python scripts/simulate_closed_loop.py <output_dir>
"""

import json
import os
import sys
from pathlib import Path

import numpy as np

# ── 电机参数 ──
TAU = 0.017             # s
V_MAX = 11.0            # 100% duty 对应电压
DEADZONE_DUTY = 20.0    # %
DEADZONE_V = 2.2        # V

# 分段线性正向模型（与逆模型匹配）
# 段1: V ∈ [0, V1]      -> rpm = 0
# 段2: V ∈ [V1, V2]     -> rpm = K2 * (V - V1)
# 段3: V ∈ [V2, Vmax]   -> rpm = K2*(V2-V1) + K3*(V-V2)

# ── 控制器参数 ──
TS_CONTROL = 0.050      # s
TS_SAMPLE = 0.010       # s

# 当前固件使用的保守参数（λ=0.05s）
# KP = 0.132
# KI = 7.770

# 当前固件使用的保守参数（λ=0.05s）
KP = 0.132
KI = 7.770

INT_LIMIT = 30.0
SEP_THRESHOLD = 10.0    # 积分分离阈值 rpm（从20降至10，改善接近稳态时的积分行为）

# ── 逆模型参数 ──
INV_V1 = 1.872
INV_V2 = 3.300
INV_K2 = 27.0322
INV_K3 = 19.1062
INV_RPM2 = INV_K2 * (INV_V2 - INV_V1)  # ≈ 38.61

# ── 实验参数 ──
PRE_SETTLE_MS = 300
STEP_HOLD_MS = 1200
DISTURB_TOTAL_MS = 2000
DISTURB_START_MS = 500
DISTURB_END_MS = 1000
REPETITIONS = 5
STEP_SETPOINTS = [60.0, 100.0, 140.0]
DISTURB_SETPOINT = 100.0
DISTURB_DUTY = 8.0      # 从15%降至8%，避免空载系统过驱动
SPEED_FILTER_ALPHA = 0.6

# ── 噪声参数 ──
ENCODER_CPR = 924.0     # counts/rev
NOISE_STD_RPM = 2.5     # 测速噪声标准差 rpm


def feedforward_voltage(rpm: float) -> float:
    rpm_abs = abs(rpm)
    if rpm_abs <= 0.0:
        v = 0.0
    elif rpm_abs <= INV_RPM2:
        v = INV_V1 + rpm_abs / INV_K2
    else:
        v = INV_V2 + (rpm_abs - INV_RPM2) / INV_K3
    return v if rpm >= 0.0 else -v


def feedforward_duty(rpm: float) -> float:
    return (feedforward_voltage(rpm) / V_MAX) * 100.0


def voltage_to_rpm_steady(v: float) -> float:
    """正向静态模型：电压 -> 稳态转速（与逆模型严格匹配）"""
    v_abs = abs(v)
    if v_abs <= INV_V1:
        rpm = 0.0
    elif v_abs <= INV_V2:
        rpm = INV_K2 * (v_abs - INV_V1)
    else:
        rpm = INV_RPM2 + INV_K3 * (v_abs - INV_V2)
    return rpm if v >= 0.0 else -rpm


def simulate_step(setpoint: float, duration_ms: int, disturb: float = 0.0,
                  seed: int = 0) -> dict:
    """仿真单次实验，返回与固件格式一致的数据字典"""
    rng = np.random.default_rng(seed)
    n_samples = duration_ms // 10
    t = np.arange(n_samples) * TS_SAMPLE

    # 离散化一阶惯性：x[k+1] = a*x[k] + (1-a)*ω_ss(V[k])
    # 其中 ω_ss 由分段线性正向模型给出
    a = np.exp(-TS_SAMPLE / TAU)

    x = 0.0  # 电机状态（转速）
    integral = 0.0
    prev_error = 0.0
    prev_output = 0.0
    first_run = True
    prev_rpm = 0.0

    time_ms = []
    sp_rpm = []
    pv_rpm = []
    duty_total = []
    duty_ff = []
    duty_fb = []
    disturbance = []

    control_counter = 0

    for i in range(n_samples):
        elapsed_ms = int(t[i] * 1000)

        # 设定值与扰动
        if disturb == 0.0:
            # 阶跃实验
            sp = 0.0 if elapsed_ms < PRE_SETTLE_MS else setpoint
            dist = 0.0
        else:
            # 扰动实验
            sp = setpoint
            dist = disturb if (DISTURB_START_MS <= elapsed_ms < DISTURB_END_MS) else 0.0

        # 测量（含噪声）
        noise = rng.normal(0.0, NOISE_STD_RPM)
        raw_rpm = x + noise
        rpm = SPEED_FILTER_ALPHA * raw_rpm + (1.0 - SPEED_FILTER_ALPHA) * prev_rpm
        prev_rpm = rpm

        # 控制计算（每 50ms）
        if control_counter % 5 == 0:
            if abs(sp) < 14.0:
                integral = 0.0
                prev_error = 0.0
                prev_output = 0.0
                first_run = True
                dt = 0.0
                ff = 0.0
                fb = 0.0
            else:
                error = sp - rpm
                proportional = KP * error

                sep_active = abs(error) > SEP_THRESHOLD
                windup_pos = prev_output >= 100.0 and error > 0.0
                windup_neg = prev_output <= -100.0 and error < 0.0

                if not sep_active and not windup_pos and not windup_neg:
                    integral += KI * error * TS_CONTROL
                    integral = np.clip(integral, -INT_LIMIT, INT_LIMIT)

                if first_run:
                    prev_error = error
                    first_run = False
                    derivative = 0.0
                else:
                    # 微分模式 OnFeedback
                    derivative = 0.0  # KD = 0

                output = proportional + integral + derivative
                output = np.clip(output, -100.0, 100.0)
                prev_output = output
                prev_error = error

                ff = feedforward_duty(sp)
                fb = output
                dt = ff + fb
                dt = np.clip(dt, -100.0, 100.0)

            control_counter = 0
        else:
            # 保持上一控制周期输出
            pass

        control_counter += 1

        # 输出PWM（含扰动）
        duty_out = np.clip(dt + dist, -100.0, 100.0)

        # 死区模拟
        if abs(duty_out) < DEADZONE_DUTY and abs(x) < 5.0:
            v_applied = 0.0
        else:
            v_applied = duty_out * V_MAX / 100.0

        # 电机更新：向当前电压对应的稳态转速指数收敛
        omega_ss = voltage_to_rpm_steady(v_applied)
        x = a * x + (1.0 - a) * omega_ss

        time_ms.append(elapsed_ms)
        sp_rpm.append(float(sp))
        pv_rpm.append(float(rpm))
        duty_total.append(float(dt))
        duty_ff.append(float(ff))
        duty_fb.append(float(fb))
        disturbance.append(float(dist))

    return {
        "time_ms": time_ms,
        "sp_rpm": sp_rpm,
        "pv_rpm": pv_rpm,
        "duty_total": duty_total,
        "duty_ff": duty_ff,
        "duty_fb": duty_fb,
        "disturbance": disturbance,
    }


def build_json(output_dir: str):
    os.makedirs(output_dir, exist_ok=True)

    meta = {
        "exp": "closed_loop",
        "sample_period_ms": 10,
        "control_period_ms": 50,
        "pre_settle_ms": PRE_SETTLE_MS,
        "step_hold_ms": STEP_HOLD_MS,
        "disturb_total_ms": DISTURB_TOTAL_MS,
        "disturb_start_ms": DISTURB_START_MS,
        "disturb_end_ms": DISTURB_END_MS,
        "reps": REPETITIONS,
        "step_setpoints": STEP_SETPOINTS,
        "disturb_duty": DISTURB_DUTY,
    }

    runs = []

    # 阶跃实验
    for sp in STEP_SETPOINTS:
        reps = []
        for rep in range(1, REPETITIONS + 1):
            data = simulate_step(sp, PRE_SETTLE_MS + STEP_HOLD_MS, seed=rep * 1000 + int(sp))
            reps.append({
                "rep": rep,
                "time_ms": data["time_ms"],
                "sp_rpm": data["sp_rpm"],
                "pv_rpm": data["pv_rpm"],
                "duty_total": data["duty_total"],
                "duty_ff": data["duty_ff"],
                "duty_fb": data["duty_fb"],
                "disturbance": data["disturbance"],
            })
        runs.append({
            "typ": "STEP",
            "sp": sp,
            "disturb": 0.0,
            "repetitions": reps,
        })

    # 扰动实验
    reps = []
    for rep in range(1, REPETITIONS + 1):
        data = simulate_step(DISTURB_SETPOINT, DISTURB_TOTAL_MS, disturb=DISTURB_DUTY, seed=rep * 2000)
        reps.append({
            "rep": rep,
            "time_ms": data["time_ms"],
            "sp_rpm": data["sp_rpm"],
            "pv_rpm": data["pv_rpm"],
            "duty_total": data["duty_total"],
            "duty_ff": data["duty_ff"],
            "duty_fb": data["duty_fb"],
            "disturbance": data["disturbance"],
        })
    runs.append({
        "typ": "DISTURB",
        "sp": DISTURB_SETPOINT,
        "disturb": DISTURB_DUTY,
        "repetitions": reps,
    })

    payload = {
        "meta": meta,
        "recorded_at": "simulated",
        "runs": runs,
    }

    json_path = Path(output_dir) / "closed_loop_data.json"
    with open(json_path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"Saved simulated JSON: {json_path}")

    # 同时保存 CSV
    csv_path = Path(output_dir) / "closed_loop_data.csv"
    with open(csv_path, "w") as f:
        f.write("typ,sp,disturb,rep,time_ms,sp_rpm,pv_rpm,duty_total,duty_ff,duty_fb,disturbance\n")
        for run in runs:
            for rep in run["repetitions"]:
                for i in range(len(rep["time_ms"])):
                    f.write(f"{run['typ']},{run['sp']},{run['disturb']},{rep['rep']},"
                            f"{rep['time_ms'][i]},{rep['sp_rpm'][i]},{rep['pv_rpm'][i]},"
                            f"{rep['duty_total'][i]},{rep['duty_ff'][i]},{rep['duty_fb'][i]},"
                            f"{rep['disturbance'][i]}\n")
    print(f"Saved simulated CSV: {csv_path}")


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "closed_loop_simulated"
    build_json(out)
    print(f"\nNext: python scripts/analyze_closed_loop.py {out}/closed_loop_data.json")
