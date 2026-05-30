#!/usr/bin/env python3
"""
空载阶跃响应实验分析脚本

功能：
    1. 读取 collect_step_response.py 保存的 JSON 数据
    2. 对每个占空比水平的多次实验做统计分析（均值、标准差）
    3. 绘制：原始曲线叠加、均值±标准差置信带、典型单条曲线
    4. 用一阶模型拟合估计电机参数：稳态增益 K、机电时间常数 τ
    5. 输出参数估计报告和拟合对比图

用法：
    python scripts/analyze_step_response.py <json_path>

要求：
    pip install numpy matplotlib scipy

电机参数模型（空载，扣除死区后）：
    Ω(s) / V_eff(s) = K / (τ·s + 1)
    其中 V_eff = V_applied - V_deadzone
"""

import json
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

# 配置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False

# ── 工程常量（来自 AGENTS.md 实测值）──
MOTOR_V_MAX = 11.0          # 100% 占空比时电机端实测电压 (V)
DEADZONE_DUTY = 20.0        # 死区占空比 (%)
DEADZONE_VOLTAGE = 2.2      # 死区电压 (V)
RPM_THRESHOLD = 2.0         # 认为电机开始转动的转速阈值 (rpm)


def duty_to_voltage(duty: float) -> float:
    """占空比 → 电机端电压（V）"""
    return duty * MOTOR_V_MAX / 100.0


def load_data(json_path: str):
    with open(json_path, "r") as f:
        payload = json.load(f)
    return payload["meta"], payload["runs"]


def compute_mean_std(runs: list, sample_period_ms: float):
    """
    对同一 duty 下的多次实验计算逐点均值和标准差。
    由于各次实验的时间戳可能不完全对齐，统一到固定时间网格。
    """
    # 找到最长的时间序列
    max_len = max(len(r["time_ms"]) for r in runs)
    time_grid = np.arange(max_len) * sample_period_ms

    rpm_matrix = np.full((len(runs), max_len), np.nan)
    for i, r in enumerate(runs):
        n = len(r["rpm"])
        rpm_matrix[i, :n] = r["rpm"]

    mean_rpm = np.nanmean(rpm_matrix, axis=0)
    std_rpm = np.nanstd(rpm_matrix, axis=0)
    return time_grid, mean_rpm, std_rpm, rpm_matrix


def estimate_steady_state(time_grid: np.ndarray, mean_rpm: np.ndarray,
                          hold_duration_ms: float, sample_period_ms: float) -> float:
    """取阶跃后最后一段的稳态平均值作为 ω_ss"""
    # 使用最后 200ms 或 hold_duration/4 的数据，取较大者
    tail_ms = max(200.0, hold_duration_ms / 4.0)
    tail_samples = int(tail_ms / sample_period_ms)
    if tail_samples < 3:
        tail_samples = 3
    return float(np.mean(mean_rpm[-tail_samples:]))


def estimate_time_constant(time_grid: np.ndarray, mean_rpm: np.ndarray,
                           omega_ss: float) -> tuple[float, float]:
    """
    63.2% 法估计时间常数 τ。
    返回 (tau_ms, t_rise_start_ms)。
    t_rise_start_ms 为转速首次超过阈值的时间（近似延迟）。
    """
    # 找到上升起点（首次超过阈值）
    above_thresh = np.where(mean_rpm > RPM_THRESHOLD)[0]
    if len(above_thresh) == 0:
        return 0.0, 0.0
    t_start_idx = above_thresh[0]
    t_start = float(time_grid[t_start_idx])

    # 63.2% 目标值
    target = 0.632 * omega_ss

    # 从 t_start 后开始找首次超过 target 的点
    candidates = np.where((time_grid >= t_start) & (mean_rpm >= target))[0]
    if len(candidates) == 0:
        return 0.0, t_start

    tau_idx = candidates[0]
    tau = float(time_grid[tau_idx]) - t_start
    return tau, t_start


def fit_first_order(time_grid: np.ndarray, mean_rpm: np.ndarray,
                    omega_ss: float, t_start: float) -> tuple[float, float] | None:
    """
    用最小二乘法拟合一阶模型：
        ω(t) = ω_ss * (1 - exp(-(t - t_start)/τ))
    返回 (tau_ms, r_squared)。
    """
    try:
        from scipy.optimize import curve_fit
    except ImportError:
        return None

    # 只使用上升段数据（t >= t_start 且未过冲）
    mask = (time_grid >= t_start) & (mean_rpm <= omega_ss * 1.05)
    t_fit = time_grid[mask]
    w_fit = mean_rpm[mask]

    if len(t_fit) < 5:
        return None

    def model(t, tau):
        return omega_ss * (1.0 - np.exp(-(t - t_start) / tau))

    try:
        popt, _ = curve_fit(model, t_fit, w_fit, p0=[20.0],
                            bounds=([1.0], [500.0]))
        tau_fit = float(popt[0])
        y_pred = model(t_fit, tau_fit)
        ss_res = np.sum((w_fit - y_pred) ** 2)
        ss_tot = np.sum((w_fit - np.mean(w_fit)) ** 2)
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0
        return tau_fit, r2
    except Exception:
        return None


def estimate_rise_time(time_grid: np.ndarray, mean_rpm: np.ndarray,
                       omega_ss: float) -> float:
    """10%-90% 上升时间（从首次超过 2 rpm 起算）"""
    above_thresh = np.where(mean_rpm > RPM_THRESHOLD)[0]
    if len(above_thresh) == 0:
        return 0.0
    t0_idx = above_thresh[0]

    t10 = None
    t90 = None
    for i in range(t0_idx, len(time_grid)):
        if t10 is None and mean_rpm[i] >= 0.10 * omega_ss:
            t10 = float(time_grid[i])
        if t90 is None and mean_rpm[i] >= 0.90 * omega_ss:
            t90 = float(time_grid[i])
            break
    if t10 is not None and t90 is not None:
        return t90 - t10
    return 0.0


def analyze_one_duty(duty: int, repetitions: list, sample_period_ms: float,
                     hold_duration_ms: float) -> dict:
    """分析单个占空比水平的所有重复实验"""
    time_grid, mean_rpm, std_rpm, rpm_matrix = compute_mean_std(
        repetitions, sample_period_ms
    )

    omega_ss = estimate_steady_state(time_grid, mean_rpm, hold_duration_ms, sample_period_ms)
    tau_632, t_start = estimate_time_constant(time_grid, mean_rpm, omega_ss)
    t_rise = estimate_rise_time(time_grid, mean_rpm, omega_ss)

    fit_result = fit_first_order(time_grid, mean_rpm, omega_ss, t_start)
    tau_fit, r2_fit = (fit_result if fit_result else (None, None))

    voltage = duty_to_voltage(duty)
    v_eff = voltage - DEADZONE_VOLTAGE
    gain_k = omega_ss / v_eff if v_eff > 0.1 else 0.0

    return {
        "duty": duty,
        "voltage": voltage,
        "v_eff": v_eff,
        "omega_ss": omega_ss,
        "tau_632_ms": tau_632,
        "tau_fit_ms": tau_fit,
        "tau_fit_r2": r2_fit,
        "t_start_ms": t_start,
        "t_rise_ms": t_rise,
        "gain_k": gain_k,
        "time_grid": time_grid,
        "mean_rpm": mean_rpm,
        "std_rpm": std_rpm,
        "rpm_matrix": rpm_matrix,
    }


def _set_xlim_adaptive(ax, time_grid: np.ndarray):
    """根据数据自适应设置 x 轴上限，保留 10% 右边距"""
    t_max = float(np.max(time_grid))
    ax.set_xlim(left=-0.02 * t_max, right=t_max * 1.10)


def plot_overlaid(results: list, output_dir: str):
    """绘制所有 duty 水平的多次实验原始曲线叠加图"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    axes = axes.flatten()

    for idx, res in enumerate(results):
        ax = axes[idx]
        duty = res["duty"]
        t = res["time_grid"]
        matrix = res["rpm_matrix"]

        for i in range(matrix.shape[0]):
            ax.plot(t, matrix[i], alpha=0.4, linewidth=0.8)

        ax.plot(t, res["mean_rpm"], "r-", linewidth=2.0, label="均值")
        ax.axhline(res["omega_ss"], color="green", linestyle="--",
                   alpha=0.7, label=f"ω_ss={res['omega_ss']:.1f}")
        ax.set_title(f"占空比 = {duty}% ({res['voltage']:.1f}V)")
        ax.set_xlabel("时间 (ms)")
        ax.set_ylabel("转速 (RPM)")
        ax.grid(True, linestyle="--", alpha=0.5)
        ax.legend(loc="lower right")
        ax.set_xlim(0, 400)

    plt.suptitle("阶跃响应：多次实验曲线叠加", fontsize=14)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    path = Path(output_dir) / "step_response_overlaid.png"
    plt.savefig(path, dpi=150)
    print(f"Saved: {path}")
    plt.close()


def plot_mean_with_std(results: list, output_dir: str):
    """绘制均值 ± 标准差置信带"""
    fig, ax = plt.subplots(figsize=(12, 7))

    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]

    for idx, res in enumerate(results):
        t = res["time_grid"]
        mean = res["mean_rpm"]
        std = res["std_rpm"]
        color = colors[idx % len(colors)]

        ax.plot(t, mean, color=color, linewidth=2.0,
                label=f"占空比 {res['duty']}% (均值)")
        ax.fill_between(t, mean - std, mean + std, color=color, alpha=0.2,
                        label=f"占空比 {res['duty']}% (±1σ)")

    ax.set_xlabel("时间 (ms)", fontsize=12)
    ax.set_ylabel("转速 (RPM)", fontsize=12)
    ax.set_title("阶跃响应：均值 ± 标准差", fontsize=14)
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.legend(loc="lower right")
    ax.set_xlim(0, 400)
    plt.tight_layout()
    path = Path(output_dir) / "step_response_mean_std.png"
    plt.savefig(path, dpi=150)
    print(f"Saved: {path}")
    plt.close()


def plot_typical_with_fit(results: list, output_dir: str):
    """绘制典型曲线（均值）与一阶拟合对比"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    axes = axes.flatten()

    for idx, res in enumerate(results):
        ax = axes[idx]
        t = res["time_grid"]
        mean = res["mean_rpm"]
        duty = res["duty"]

        ax.plot(t, mean, "b-", linewidth=2.0, label="实测（均值）")

        # 绘制一阶拟合曲线
        if res["tau_fit_ms"] is not None:
            tau = res["tau_fit_ms"]
            t0 = res["t_start_ms"]
            y_model = res["omega_ss"] * (1.0 - np.exp(-(t - t0) / tau))
            y_model[t < t0] = 0.0
            ax.plot(t, y_model, "r--", linewidth=2.0,
                    label=f"一阶拟合 (τ={tau:.0f}ms, R^2={res['tau_fit_r2']:.3f})")

        ax.axhline(res["omega_ss"], color="green", linestyle=":", alpha=0.7)
        ax.axvline(res["t_start_ms"], color="gray", linestyle=":", alpha=0.5)

        ax.set_title(f"占空比 {duty}% — K={res['gain_k']:.1f} rpm/V, "
                     f"t_rise={res['t_rise_ms']:.0f}ms")
        ax.set_xlabel("时间 (ms)")
        ax.set_ylabel("转速 (RPM)")
        ax.grid(True, linestyle="--", alpha=0.5)
        ax.legend(loc="lower right")
        ax.set_xlim(0, 400)

    plt.suptitle("阶跃响应：实测曲线与一阶模型拟合", fontsize=14)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    path = Path(output_dir) / "step_response_fit.png"
    plt.savefig(path, dpi=150)
    print(f"Saved: {path}")
    plt.close()


def plot_parameter_summary(results: list, output_dir: str):
    """绘制参数估计汇总图：K 和 τ 随电压的变化"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    duties = [r["duty"] for r in results]
    voltages = [r["voltage"] for r in results]
    gains = [r["gain_k"] for r in results]
    taus = [r["tau_fit_ms"] if r["tau_fit_ms"] else r["tau_632_ms"] for r in results]

    # 增益 K
    ax1.plot(voltages, gains, "bo-", markersize=8, linewidth=1.5)
    ax1.set_xlabel("施加电压 (V)", fontsize=12)
    ax1.set_ylabel("稳态增益 K (rpm/V)", fontsize=12)
    ax1.set_title("稳态增益 vs 电压", fontsize=13)
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.set_ylim(bottom=0)
    for d, v, k in zip(duties, voltages, gains):
        ax1.annotate(f"{d}%", (v, k), textcoords="offset points",
                     xytext=(0, 10), ha="center", fontsize=9)

    # 时间常数 τ
    ax2.plot(voltages, taus, "rs-", markersize=8, linewidth=1.5)
    ax2.set_xlabel("施加电压 (V)", fontsize=12)
    ax2.set_ylabel("时间常数 τ (ms)", fontsize=12)
    ax2.set_title("时间常数 vs 电压", fontsize=13)
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.set_ylim(bottom=0)
    for d, v, t in zip(duties, voltages, taus):
        ax2.annotate(f"{d}%", (v, t), textcoords="offset points",
                     xytext=(0, 10), ha="center", fontsize=9)

    plt.suptitle("电机参数估计汇总", fontsize=14)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    path = Path(output_dir) / "step_response_params.png"
    plt.savefig(path, dpi=150)
    print(f"Saved: {path}")
    plt.close()


def print_report(results: list):
    """打印参数估计报告"""
    print("\n" + "=" * 70)
    print("           空载阶跃响应实验 —— 电机参数估计报告")
    print("=" * 70)
    print(f"\n工程常量（来自 AGENTS.md）：")
    print(f"  死区电压 V_d = {DEADZONE_VOLTAGE:.1f} V (duty ≥ {DEADZONE_DUTY:.0f}%)")
    print(f"  满压电压 V_max = {MOTOR_V_MAX:.1f} V (100% duty)")

    print(f"\n{'Duty':>6} {'Voltage':>8} {'V_eff':>8} {'ω_ss':>10} "
          f"{'t_rise':>8} {'τ_63.2':>8} {'τ_fit':>8} {'R²':>6} {'K':>10}")
    print("-" * 78)

    all_k = []
    all_tau = []
    all_rise = []

    for res in results:
        tau_fit_str = f"{res['tau_fit_ms']:.0f}" if res["tau_fit_ms"] else "N/A"
        r2_str = f"{res['tau_fit_r2']:.3f}" if res["tau_fit_r2"] is not None else "N/A"
        print(f"{res['duty']:>6}% {res['voltage']:>8.2f} {res['v_eff']:>8.2f} "
              f"{res['omega_ss']:>10.1f} {res['t_rise_ms']:>8.0f} "
              f"{res['tau_632_ms']:>8.0f} {tau_fit_str:>8} {r2_str:>6} {res['gain_k']:>10.1f}")

        all_k.append(res["gain_k"])
        tau_val = res["tau_fit_ms"] if res["tau_fit_ms"] else res["tau_632_ms"]
        if tau_val and tau_val > 0:
            all_tau.append(tau_val)
        if res["t_rise_ms"] > 0:
            all_rise.append(res["t_rise_ms"])

    print("-" * 70)

    if all_k:
        k_mean = np.mean(all_k)
        k_std = np.std(all_k)
        print(f"\n稳态增益 K = {k_mean:.1f} ± {k_std:.1f} rpm/V  "
              f"(CV = {k_std/k_mean*100:.1f}%)")

    if all_tau:
        tau_mean = np.mean(all_tau)
        tau_std = np.std(all_tau)
        print(f"时间常数 τ = {tau_mean:.0f} ± {tau_std:.0f} ms  "
              f"(CV = {tau_std/tau_mean*100:.1f}%)")
        print(f"           = {tau_mean/1000:.3f} ± {tau_std/1000:.3f} s")

    if all_rise:
        rise_mean = np.mean(all_rise)
        rise_std = np.std(all_rise)
        print(f"上升时间 t_rise(10%-90%) = {rise_mean:.0f} ± {rise_std:.0f} ms  "
              f"(CV = {rise_std/rise_mean*100:.1f}%)")

    # 一阶模型表达式
    if all_k and all_tau:
        print(f"\n┌─────────────────────────────────────────────────────────────────────┐")
        print(f"│  空载电机一阶模型（扣除死区后）：                                    │")
        print(f"│                                                                     │")
        print(f"│      Ω(s)        {k_mean:.1f}                                         │")
        print(f"│    ─────── = ───────────────                                        │")
        print(f"│    V_eff(s)   {tau_mean/1000:.4f}·s + 1                                    │")
        print(f"│                                                                     │")
        print(f"│  其中 V_eff = V_applied - {DEADZONE_VOLTAGE:.1f} V (死区电压)                    │")
        print(f"└─────────────────────────────────────────────────────────────────────┘")

    print("=" * 78)


def main():
    if len(sys.argv) < 2:
        print("Usage: python scripts/analyze_step_response.py <json_path>")
        sys.exit(1)

    json_path = sys.argv[1]
    output_dir = Path(json_path).parent

    meta, runs = load_data(json_path)
    sample_period_ms = float(meta.get("sample_period_ms", 50))
    hold_duration_ms = float(meta.get("hold_ms", 3000))

    print(f"Loaded {len(runs)} duty levels from {json_path}")

    # 逐个 duty 分析
    results = []
    for run_group in runs:
        duty = run_group["duty"]
        repetitions = run_group["repetitions"]
        print(f"  Analyzing duty={duty}% with {len(repetitions)} repetitions...")
        res = analyze_one_duty(duty, repetitions, sample_period_ms, hold_duration_ms)
        results.append(res)

    # 按 duty 排序
    results.sort(key=lambda x: x["duty"])

    # 打印报告
    print_report(results)

    # 绘图
    print("\nGenerating plots...")
    plot_overlaid(results, output_dir)
    plot_mean_with_std(results, output_dir)
    plot_typical_with_fit(results, output_dir)
    plot_parameter_summary(results, output_dir)

    print(f"\nAll outputs saved to: {output_dir}")


if __name__ == "__main__":
    main()
