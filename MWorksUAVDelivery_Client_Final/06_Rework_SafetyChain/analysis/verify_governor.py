"""
验证参考调节器（时间最优跟踪微分器 TD）设计。

验证链路是完整闭合的，不需要整机物理模型：
    参考跳变 -> PID3 的微分项踢 -> 倾角指令阶跃 -> 姿态尖峰
前三环节我都能精确复算（控制器拓扑已完全掌握）。所以只要证明
"整形后倾角指令不再出现阶跃"，姿态尖峰的激励源就消失了。

TD 形式（连续）:
    der(r) = v
    der(v) = -aMax * tanh( (r - u + v*|v|/(2*aMax)) / delta )
斜坡输入下 v 收敛到斜率、a->0，故无稳态滞后；阶跃下按 aMax 限加速度过渡。
"""
import numpy as np
import pandas as pd
from ident_attitude import SIM, DEG, reconstruct_tilt_cmd, hybrid_smc_out, pid_out


def td_governor(u, t, aMax, vMax, delta=0.05):
    r = np.zeros_like(u)
    v = np.zeros_like(u)
    r[0] = u[0]
    for k in range(1, len(u)):
        dt = t[k] - t[k - 1]
        s = (r[k - 1] - u[k - 1]) + v[k - 1] * abs(v[k - 1]) / (2 * aMax)
        a = -aMax * np.tanh(s / delta)
        vn = np.clip(v[k - 1] + a * dt, -vMax, vMax)
        r[k] = r[k - 1] + vn * dt
        v[k] = vn
    return r, v


def tilt_cmd_from_ref(ref, pos, t, variant):
    """用已复刻的控制器通道，从(参考,实测位置)算倾角指令。"""
    e = ref - pos
    if variant == "baseline":
        o = pid_out(e, t, 1.5, 0.0, 1.0)
        lim = 15.0 * DEG
    else:
        o = hybrid_smc_out(e, t, 2.0, 0.0, 1.35, 1.25, 0.1, 0.35, 2.1)
        lim = 9.5 * DEG
    return np.clip(0.1 * o, -lim, lim)


def max_rate(x, t):
    return np.abs(np.diff(x) / np.diff(t)).max()


print("=" * 92)
print("参考调节器 (TD) 效果：aMax=4 m/s^2, vMax=4 m/s, delta=0.05")
print("=" * 92)
AMAX, VMAX = 4.0, 4.0

for sc, va in [("climb", "enhanced"), ("spiral", "enhanced"), ("eight", "enhanced")]:
    df = pd.read_csv(SIM / f"{sc}_{va}.csv")
    t = df["time"].to_numpy(float)
    print(f"\n---- {sc}_{va} ----")
    for axis, poscol in [("ref_x", "pos_x"), ("ref_y", "pos_y")]:
        raw = df[axis].to_numpy(float)
        pos = df[poscol].to_numpy(float)
        gov, _ = td_governor(raw, t, AMAX, VMAX)

        cmd_raw = tilt_cmd_from_ref(raw, pos, t, va)
        cmd_gov = tilt_cmd_from_ref(gov, pos, t, va)

        # 整形引入的额外跟踪误差（对原始参考而言的滞后）
        lag = np.abs(gov - raw)
        # 只看非跳变段的滞后（跳变段本来就不可行）
        jump_t = {"climb": 30.0, "spiral": 10.0, "eight": 10.0}[sc]
        steady = (t < jump_t - 0.5) | (t > jump_t + 2.0)

        print(f"  {axis}: 参考最大变化率 {max_rate(raw, t):8.2f} -> {max_rate(gov, t):6.2f} m/s | "
              f"倾角指令最大变化率 {max_rate(cmd_raw, t)/DEG:9.1f} -> {max_rate(cmd_gov, t)/DEG:7.1f} deg/s | "
              f"平稳段滞后 max {lag[steady].max():.4f} m  rms {np.sqrt(np.mean(lag[steady]**2)):.4f} m")

print("\n" + "=" * 92)
print("对姿态超限的直接影响：超限时刻附近的倾角指令阶跃是否被消除")
print("=" * 92)
for sc, va, tj in [("climb", "enhanced", 30.0), ("spiral", "enhanced", 10.0)]:
    df = pd.read_csv(SIM / f"{sc}_{va}.csv")
    t = df["time"].to_numpy(float)
    dt = t[1] - t[0]
    w = (t >= tj - 0.05) & (t <= tj + 0.40)

    res = {}
    for axis, poscol, name in [("ref_x", "pos_x", "pitch"), ("ref_y", "pos_y", "roll")]:
        raw = df[axis].to_numpy(float)
        pos = df[poscol].to_numpy(float)
        gov, _ = td_governor(raw, t, AMAX, VMAX)
        res[name] = (tilt_cmd_from_ref(raw, pos, t, va)[w],
                     tilt_cmd_from_ref(gov, pos, t, va)[w])

    print(f"\n---- {sc} 跳变窗口 t={tj-0.05:.2f}~{tj+0.40:.2f} ----")
    print("   指令峰值 (deg)          原始      整形后")
    for name in ["pitch", "roll"]:
        a, b = res[name]
        print(f"   {name}_cmd 峰值        {np.abs(a).max()/DEG:8.2f}  {np.abs(b).max()/DEG:8.2f}")
        print(f"   {name}_cmd 最大变化率 {max_rate(a, t[w])/DEG:8.1f}  {max_rate(b, t[w])/DEG:8.1f}  deg/s")

print("\n" + "=" * 92)
print("aMax 敏感度（climb ref_y，超限主因通道）")
print("=" * 92)
df = pd.read_csv(SIM / "climb_enhanced.csv")
t = df["time"].to_numpy(float)
raw, pos = df["ref_y"].to_numpy(float), df["pos_y"].to_numpy(float)
jump = (t >= 29.95) & (t <= 30.5)
steady = (t < 29.5) | (t > 32.0)
print(" aMax   倾角指令峰值  指令最大变化率   平稳段滞后rms")
for am in [1.0, 2.0, 3.0, 4.0, 6.0, 10.0, 20.0]:
    gov, _ = td_governor(raw, t, am, VMAX)
    c = tilt_cmd_from_ref(gov, pos, t, "enhanced")
    lag = np.abs(gov - raw)
    print(f" {am:5.1f}   {np.abs(c[jump]).max()/DEG:9.2f}deg  {max_rate(c[jump], t[jump])/DEG:11.1f}deg/s"
          f"   {np.sqrt(np.mean(lag[steady]**2)):10.5f} m")
