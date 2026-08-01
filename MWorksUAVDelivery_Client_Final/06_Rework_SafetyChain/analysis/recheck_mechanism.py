"""
独立复核对抗验证提出的三条关键反驳，不复用其脚本：
 [A] climb t=20（单轴折角）vs t=30（双轴同时折角）：指令是否等价、姿态峰是否差异显著
 [B] eight t=10（三轴同时不连续）：是否构成"双轴同时=超限"的反例
 [C] 混控层面：t=30 是否出现 pitch/roll 内环同时饱和 + X型混控在对角旋翼同号叠加
"""
import numpy as np
import pandas as pd
from ident_attitude import SIM, DEG, reconstruct_tilt_cmd

def win(t, t0, a=-0.05, b=0.40):
    return (t >= t0 + a) & (t <= t0 + b)

def peak_stats(df, va, t0):
    t = df["time"].to_numpy(float)
    pc, rc = reconstruct_tilt_cmd(df, va)
    w = win(t, t0)
    pit = df["pitch"].to_numpy(float)[w] / DEG
    rol = df["roll"].to_numpy(float)[w] / DEG
    cp, cr = pc[w] / DEG, rc[w] / DEG
    tt = t[w]
    # 上升时间：从 10% 到峰值
    def rise(sig):
        k = int(np.argmax(np.abs(sig)))
        pk = sig[k]
        thr = 0.1 * abs(pk)
        j = k
        while j > 0 and abs(sig[j]) > thr:
            j -= 1
        return tt[k] - tt[j], abs(pk)
    rt_p, cpk = rise(cp)
    rate_p = np.abs(np.diff(cp) / np.diff(tt)).max()
    rate_r = np.abs(np.diff(cr) / np.diff(tt)).max()
    return dict(
        cmd_pitch_pk=cpk, cmd_roll_pk=np.abs(cr).max(),
        cmd_rise_s=rt_p, cmd_rate_p=rate_p, cmd_rate_r=rate_r,
        att_pitch_pk=np.abs(pit).max(), att_roll_pk=np.abs(rol).max(),
        amp_pitch=np.abs(pit).max() / max(cpk, 1e-9),
        att0_p=abs(pit[0]), att0_r=abs(rol[0]),
    )

print("=" * 100)
print("[A] climb_enhanced：t=20.00（仅 ref_x 折角） vs t=30.00（ref_x 与 ref_y 同时折角）")
print("=" * 100)
df = pd.read_csv(SIM / "climb_enhanced.csv")
t = df["time"].to_numpy(float)
# 先确认两个时刻的参考不连续构成
for t0 in (20.0, 30.0, 40.0):
    k = int(np.argmin(np.abs(t - t0)))
    d = {c: (df[c].to_numpy(float)[k + 1] - df[c].to_numpy(float)[k]) / (t[k + 1] - t[k])
         for c in ("ref_x", "ref_y", "ref_z")}
    pre = {c: (df[c].to_numpy(float)[k] - df[c].to_numpy(float)[k - 1]) / (t[k] - t[k - 1])
           for c in ("ref_x", "ref_y", "ref_z")}
    print(f"  t={t0:5.1f}s 参考速度 前->后:  " +
          "  ".join(f"{c[-1]}: {pre[c]:+.3f}->{d[c]:+.3f}" for c in ("ref_x", "ref_y", "ref_z")))
print()
for t0 in (20.0, 30.0, 40.0):
    s = peak_stats(df, "enhanced", t0)
    print(f"  t={t0:5.1f}s  指令峰 pitch={s['cmd_pitch_pk']:6.3f}° roll={s['cmd_roll_pk']:6.3f}°"
          f"  上升{s['cmd_rise_s']*1000:5.1f}ms  速率 p={s['cmd_rate_p']:6.1f} r={s['cmd_rate_r']:6.1f}°/s"
          f" | 姿态峰 pitch={s['att_pitch_pk']:6.3f}° roll={s['att_roll_pk']:6.3f}°"
          f"  放大={s['amp_pitch']:.3f}  {'超限' if max(s['att_pitch_pk'],s['att_roll_pk'])>15 else '合规'}")

print()
print("=" * 100)
print("[B] eight_enhanced t=10.00：三轴同时不连续，是否为反例")
print("=" * 100)
de = pd.read_csv(SIM / "eight_enhanced.csv")
te = de["time"].to_numpy(float)
k = int(np.argmin(np.abs(te - 10.0)))
for c in ("ref_x", "ref_y", "ref_z"):
    v = de[c].to_numpy(float)
    print(f"  {c}: 前速度 {(v[k]-v[k-1])/(te[k]-te[k-1]):+8.4f}  后速度 {(v[k+1]-v[k])/(te[k+1]-te[k]):+8.4f} m/s")
s = peak_stats(de, "enhanced", 10.0)
print(f"  指令峰 pitch={s['cmd_pitch_pk']:.3f}° roll={s['cmd_roll_pk']:.3f}°"
      f" | 姿态峰 pitch={s['att_pitch_pk']:.3f}° roll={s['att_roll_pk']:.3f}° 放大={s['amp_pitch']:.3f}"
      f"  {'超限' if max(s['att_pitch_pk'],s['att_roll_pk'])>15 else '合规'}")

print()
print("=" * 100)
print("[C] 混控层：电机指令摆幅（X型混控下 pitch/roll 同时饱和是否在对角旋翼同号叠加）")
print("=" * 100)
print("  混控: y=+(m1+T) y1=-(m2+T) y2=+(m3+T) y3=-(m4+T)")
print("        m1=-gy-gp+gr  m2=+gy-gp-gr  m3=-gy+gp-gr  m4=+gy+gp+gr")
for tag, d0, t0 in [("climb t=20", df, 20.0), ("climb t=30", df, 30.0),
                    ("climb t=40", df, 40.0), ("eight t=10", de, 10.0)]:
    tt = d0["time"].to_numpy(float)
    w = win(tt, t0)
    u = d0[["u1", "u2", "u3", "u4"]].to_numpy(float)[w]
    span = u.max(0) - u.min(0)
    print(f"  {tag:12s} u摆幅 = " + "  ".join(f"u{i+1}={span[i]:7.3f}" for i in range(4))
          + f"   最大/最小 = {span.max()/max(span.min(),1e-9):6.2f}")

print()
print("=" * 100)
print("[D] 反解内环通道，检验 pitch/roll 内环是否同时饱和在 ±6.1")
print("=" * 100)
# 由 u 反解: T,Y,P,R  (mixing 正交)
for tag, d0, t0 in [("climb t=20", df, 20.0), ("climb t=30", df, 30.0), ("eight t=10", de, 10.0)]:
    tt = d0["time"].to_numpy(float); w = win(tt, t0)
    u = d0[["u1", "u2", "u3", "u4"]].to_numpy(float)[w]
    y, y1, y2, y3 = u[:, 0], u[:, 1], u[:, 2], u[:, 3]
    m1, m2, m3, m4 = y, -y1, y2, -y3           # m_i + T
    T = (m1 + m2 + m3 + m4) / 4.0
    a1, a2, a3, a4 = m1 - T, m2 - T, m3 - T, m4 - T
    gy = (-a1 + a2 - a3 + a4) / 4.0
    gp = (-a1 - a2 + a3 + a4) / 4.0
    gr = (a1 - a2 - a3 + a4) / 4.0
    P = gp / 0.707; R = gr / 0.707
    print(f"  {tag:12s} |P|max={np.abs(P).max():7.4f}  |R|max={np.abs(R).max():7.4f}"
          f"  (限幅 6.1)  同时饱和(both>6.05)的采样点 = {int(((np.abs(P)>6.05)&(np.abs(R)>6.05)).sum())}")
