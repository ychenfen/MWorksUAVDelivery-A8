"""放大看超限时刻附近，确认参考轨迹是否存在阶跃不连续。"""
import numpy as np
import pandas as pd
from ident_attitude import SIM, DEG, reconstruct_tilt_cmd

for sc, va, t0 in [("climb", "enhanced", 30.0), ("climb", "baseline", 30.0),
                   ("spiral", "enhanced", 10.0), ("spiral", "baseline", 10.0)]:
    df = pd.read_csv(SIM / f"{sc}_{va}.csv")
    t = df["time"].to_numpy(float)
    pc, rc = reconstruct_tilt_cmd(df, va)
    dt = t[1] - t[0]
    i0, i1 = int((t0 - 0.10) / dt), int((t0 + 0.16) / dt)
    print(f"\n===== {sc}_{va}  t={t0-0.1:.2f}~{t0+0.15:.2f} =====")
    print("  t      ref_x    ref_y    ref_z |  d(ref_x)  d(ref_y) | p_cmd  pitch   roll |   u1     u2")
    for i in range(i0, min(i1, len(t))):
        dx = (df['ref_x'][i] - df['ref_x'][i-1]) if i else 0.0
        dy = (df['ref_y'][i] - df['ref_y'][i-1]) if i else 0.0
        mark = "  <<<" if abs(df['pitch'][i]) > 15*DEG or abs(df['roll'][i]) > 15*DEG else ""
        print(f"{t[i]:6.2f} {df['ref_x'][i]:8.3f} {df['ref_y'][i]:8.3f} {df['ref_z'][i]:8.3f} |"
              f" {dx:9.4f} {dy:9.4f} | {pc[i]/DEG:6.2f} {df['pitch'][i]/DEG:6.2f} {df['roll'][i]/DEG:6.2f} |"
              f" {df['u1'][i]:6.2f} {df['u2'][i]:6.2f}{mark}")

# 全程找参考轨迹的跳变点
print("\n\n===== 参考轨迹跳变扫描 (|Δref| > 0.05 m/step) =====")
for sc in ["climb", "spiral", "eight"]:
    df = pd.read_csv(SIM / f"{sc}_enhanced.csv")
    t = df["time"].to_numpy(float)
    for ax in ["ref_x", "ref_y", "ref_z"]:
        d = np.abs(np.diff(df[ax].to_numpy(float)))
        idx = np.where(d > 0.05)[0]
        if len(idx):
            print(f"{sc:7s} {ax}: {len(idx)} 处跳变, 最大 {d.max():.4f} m/step, "
                  f"时刻 {np.array2string(t[idx+1][:8], precision=2)}")
        else:
            print(f"{sc:7s} {ax}: 连续 (最大步进 {d.max():.5f} m)")
