"""
选 ERG 限速值：必须大于正常飞行所需，小于跳变激励。
同时给出 CBF 的角速率增益量级（从实测姿态角速率反推制动能力）。
"""
import numpy as np
import pandas as pd
from ident_attitude import SIM, DEG, reconstruct_tilt_cmd

JUMP = {"climb": [20.0, 30.0, 5.0], "spiral": [10.0], "eight": [10.0]}

print("倾角指令变化率分布 (deg/s)，剔除跳变后 0.5s 窗口")
print("-" * 88)
print(f"{'场景':16s}{'p50':>9}{'p90':>9}{'p99':>9}{'p99.9':>9}{'最大':>10}   {'跳变段最大':>12}")
for sc in ["climb", "spiral", "eight"]:
    for va in ["baseline", "enhanced"]:
        df = pd.read_csv(SIM / f"{sc}_{va}.csv")
        t = df["time"].to_numpy(float)
        pc, rc = reconstruct_tilt_cmd(df, va)
        tm = 0.5 * (t[1:] + t[:-1])
        rate = np.abs(np.diff(np.concatenate([pc, rc]).reshape(2, -1), axis=1) / np.diff(t)).max(axis=0)
        injump = np.zeros_like(tm, dtype=bool)
        for tj in JUMP[sc]:
            injump |= (tm >= tj - 0.02) & (tm <= tj + 0.5)
        norm = rate[~injump] / DEG
        jm = rate[injump] / DEG
        print(f"{sc+'_'+va:16s}{np.percentile(norm,50):9.1f}{np.percentile(norm,90):9.1f}"
              f"{np.percentile(norm,99):9.1f}{np.percentile(norm,99.9):9.1f}{norm.max():10.1f}   {jm.max():12.1f}")

print("\n\n实测姿态角速率（用于 CBF 制动裕度 k 的量级）")
print("-" * 88)
for sc in ["climb", "spiral", "eight"]:
    for va in ["baseline", "enhanced"]:
        df = pd.read_csv(SIM / f"{sc}_{va}.csv")
        t = df["time"].to_numpy(float)
        for ang in ["roll", "pitch"]:
            a = df[ang].to_numpy(float)
            w = np.diff(a) / np.diff(t)
            k = int(np.argmax(np.abs(a)))
            # 峰值时刻前的角速率 -> 制动裕度 k ~= 峰值超出量 / 该角速率
            print(f"  {sc+'_'+va:16s} {ang:5s}: |w|max={np.abs(w).max()/DEG:7.1f}deg/s  "
                  f"峰值={a[k]/DEG:+7.2f}deg  峰值前20ms角速率={(w[max(k-2,0)])/DEG:+8.1f}deg/s")

print("\n\n结论用数据：把 ERG 限速设为 X，正常飞行不受影响的比例")
print("-" * 88)
for X in [40, 60, 80, 100, 150, 200, 300]:
    binds = []
    for sc in ["climb", "spiral", "eight"]:
        df = pd.read_csv(SIM / f"{sc}_enhanced.csv")
        t = df["time"].to_numpy(float)
        pc, rc = reconstruct_tilt_cmd(df, "enhanced")
        tm = 0.5 * (t[1:] + t[:-1])
        rate = np.abs(np.diff(np.concatenate([pc, rc]).reshape(2, -1), axis=1) / np.diff(t)).max(axis=0)
        injump = np.zeros_like(tm, dtype=bool)
        for tj in JUMP[sc]:
            injump |= (tm >= tj - 0.02) & (tm <= tj + 0.5)
        binds.append((rate[~injump] / DEG > X).mean() * 100)
    print(f"  限速 {X:3d} deg/s -> 正常飞行中被限住的采样点占比: "
          + "  ".join(f"{s}={b:5.2f}%" for s, b in zip(["climb", "spiral", "eight"], binds)))
