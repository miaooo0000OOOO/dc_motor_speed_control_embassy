#!/usr/bin/env python3
"""
绘制闭环实验中文图

用法:
    python scripts/plot_closed_loop.py [json_path] [output_dir]
"""

import json
import sys
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False


def load_data(path):
    with open(path) as f:
        return json.load(f)


def get_mean_std(reps, key):
    mats = np.array([r[key] for r in reps])
    return np.mean(mats, axis=0), np.std(mats, axis=0)


def plot_step(data, output_dir):
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    axes = axes.flatten()

    step_runs = [r for r in data['runs'] if r['typ'] == 'STEP']
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c']

    for idx, run in enumerate(step_runs):
        ax = axes[idx]
        sp = run['sp']
        t = np.array(run['repetitions'][0]['time_ms'])
        pv_mean, pv_std = get_mean_std(run['repetitions'], 'pv_rpm')

        # 找到阶跃时刻
        sp_arr = np.array(run['repetitions'][0]['sp_rpm'])
        step_idx = np.where(sp_arr > 0)[0][0]
        t_step = t[step_idx]
        t_rel = t - t_step

        # 10%-90% 上升时间
        pv_10 = 0.1 * sp
        pv_90 = 0.9 * sp
        idx_10 = step_idx + np.where(pv_mean[step_idx:] >= pv_10)[0]
        idx_90 = step_idx + np.where(pv_mean[step_idx:] >= pv_90)[0]
        t_rise = t[idx_90[0]] - t[idx_10[0]] if len(idx_10) > 0 and len(idx_90) > 0 else np.nan

        # 超调
        pv_max = np.max(pv_mean[step_idx:])
        overshoot = (pv_max - sp) / sp * 100 if sp > 0 else 0

        # 稳态误差（最后 500ms）
        ess = np.mean(pv_mean[t >= (t[-1] - 500)]) - sp

        ax.plot(t, pv_mean, color=colors[idx], linewidth=2.0, label='实测（均值）')
        ax.fill_between(t, pv_mean - pv_std, pv_mean + pv_std, color=colors[idx], alpha=0.2, label='±1σ')
        ax.axhline(sp, color='red', linestyle='--', linewidth=1.5, label=f'设定值={sp:.0f}')

        ax.set_title(f'阶跃响应：设定值 = {sp:.0f} rpm', fontsize=13)
        ax.set_xlabel('时间 (ms)', fontsize=12)
        ax.set_ylabel('转速 (RPM)', fontsize=12)
        ax.grid(True, linestyle='--', alpha=0.4)
        ax.legend(loc='lower right', fontsize=9)

        # 标注
        text = f'$t_r$={t_rise:.0f}ms\n$\\sigma$={overshoot:.1f}%\n$e_{{ss}}$={ess:+.1f}rpm'
        ax.text(0.98, 0.25, text, transform=ax.transAxes, fontsize=10,
                verticalalignment='top', horizontalalignment='right',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # 隐藏第4个子图
    axes[3].axis('off')

    plt.suptitle('闭环阶跃响应（前馈 + PI）', fontsize=15)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    path = f'{output_dir}/closed_loop_step.png'
    plt.savefig(path, dpi=150, bbox_inches='tight')
    print(f'Saved: {path}')
    plt.close()


def plot_disturbance(data, output_dir):
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), gridspec_kw={'height_ratios': [2, 1]})

    run = [r for r in data['runs'] if r['typ'] == 'DISTURB'][0]
    t = np.array(run['repetitions'][0]['time_ms'])
    sp = run['sp']

    pv_mean, pv_std = get_mean_std(run['repetitions'], 'pv_rpm')
    ff_mean, _ = get_mean_std(run['repetitions'], 'duty_ff')
    fb_mean, _ = get_mean_std(run['repetitions'], 'duty_fb')
    dist = np.array(run['repetitions'][0]['disturbance'])

    # 扰动起止
    disturb_on_idx = np.where(dist > 0)[0][0]
    disturb_off_idx = np.where(dist > 0)[0][-1]
    t_on = t[disturb_on_idx]
    t_off = t[disturb_off_idx]

    # 扰动期间最大偏差
    during_mask = (t >= t_on) & (t <= t_off)
    max_dev = np.max(np.abs(pv_mean[during_mask] - sp))

    # 恢复时间
    post_mask = t > t_off
    band = 0.05 * sp
    t_recover = np.nan
    for i in range(disturb_off_idx, len(pv_mean)):
        if np.all(np.abs(pv_mean[i:] - sp) <= band):
            t_recover = t[i] - t_off
            break

    # 反馈峰值
    fb_peak = np.max(np.abs(fb_mean[during_mask]))

    ax = axes[0]
    ax.plot(t, pv_mean, 'b-', linewidth=2.0, label='实际转速')
    ax.fill_between(t, pv_mean - pv_std, pv_mean + pv_std, color='blue', alpha=0.15)
    ax.axhline(sp, color='red', linestyle='--', linewidth=1.5, label=f'设定值={sp:.0f}')
    ax.axvline(t_on, color='orange', linestyle='-.', alpha=0.7, label='扰动施加')
    ax.axvline(t_off, color='green', linestyle='-.', alpha=0.7, label='扰动撤销')
    ax.set_ylabel('转速 (RPM)', fontsize=12)
    ax.set_title(f'扰动抑制：{t_on:.0f}ms 时施加 +{run["disturb"]:.0f}% 占空比扰动', fontsize=13)
    ax.grid(True, linestyle='--', alpha=0.4)
    ax.legend(loc='upper right', fontsize=9)

    text = f'最大偏差 = {max_dev:.1f} rpm\n恢复时间 = {t_recover:.0f} ms\n反馈峰值 = {fb_peak:.1f} %'
    ax.text(0.98, 0.05, text, transform=ax.transAxes, fontsize=10,
            verticalalignment='bottom', horizontalalignment='right',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    ax = axes[1]
    ax.plot(t, ff_mean, 'g-', linewidth=2.0, label='前馈')
    ax.plot(t, fb_mean, 'r-', linewidth=2.0, label='反馈')
    ax.plot(t, dist, 'm--', linewidth=1.5, label='扰动')
    ax.set_xlabel('时间 (ms)', fontsize=12)
    ax.set_ylabel('占空比 (%)', fontsize=12)
    ax.grid(True, linestyle='--', alpha=0.4)
    ax.legend(loc='upper right', fontsize=9)

    plt.suptitle('闭环扰动响应与控制量分解', fontsize=15)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    path = f'{output_dir}/closed_loop_disturbance.png'
    plt.savefig(path, dpi=150, bbox_inches='tight')
    print(f'Saved: {path}')
    plt.close()


def plot_control_decomp(data, output_dir):
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    step_runs = [r for r in data['runs'] if r['typ'] == 'STEP']

    for idx, run in enumerate(step_runs):
        ax = axes[idx]
        sp = run['sp']
        t = np.array(run['repetitions'][0]['time_ms'])
        dt_mean, _ = get_mean_std(run['repetitions'], 'duty_total')
        ff_mean, _ = get_mean_std(run['repetitions'], 'duty_ff')
        fb_mean, _ = get_mean_std(run['repetitions'], 'duty_fb')

        ax.plot(t, dt_mean, 'b-', linewidth=2.0, label='总输出')
        ax.plot(t, ff_mean, 'g--', linewidth=2.0, label='前馈')
        ax.plot(t, fb_mean, 'r--', linewidth=2.0, label='反馈')

        ax.set_title(f'设定值 = {sp:.0f} rpm', fontsize=13)
        ax.set_xlabel('时间 (ms)', fontsize=12)
        ax.set_ylabel('占空比 (%)', fontsize=12)
        ax.grid(True, linestyle='--', alpha=0.4)
        ax.legend(loc='upper right', fontsize=9)

    plt.suptitle('控制量分解（前馈 + 反馈）', fontsize=15)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    path = f'{output_dir}/closed_loop_control_decomp.png'
    plt.savefig(path, dpi=150, bbox_inches='tight')
    print(f'Saved: {path}')
    plt.close()


def main():
    json_path = sys.argv[1] if len(sys.argv) > 1 else 'closed_loop_20260527_1028/closed_loop_data.json'
    output_dir = sys.argv[2] if len(sys.argv) > 2 else 'closed_loop_20260527_1028'

    data = load_data(json_path)
    plot_step(data, output_dir)
    plot_disturbance(data, output_dir)
    plot_control_decomp(data, output_dir)


if __name__ == '__main__':
    main()
