"""
从 MWorks 真实仿真输出反推倾角指令，辨识姿态内环闭环特性。

控制器拓扑（来自 QuadrotorModel.mo Blocks.Controller.Controller）:
    x 通道: ref_x - pos_x -> PID3 -> gain6(0.1) -> limiter1 -> pitch_cmd
            feedback1 = pitch_cmd - angle[2](pitch)
    y 通道: ref_y - pos_y -> PID4 -> gain5(0.1) -> limiter2 -> roll_cmd
            feedback2 = roll_cmd - gain13(-1)*angle[1] = roll_cmd + roll

baseline : PID3/PID4 = PID(KP=1.5, KI=0, KD=1),            limiter = 15 deg
enhanced : PID3/PID4 = HybridPIDSlidingMode(KP=2.0, KI=0, KD=1.35,
                        lambda=1.25, kSwitch=0.1, boundary=0.35, uMax=2.1),
                                                            limiter = 9.5 deg

Modelica.Blocks.Continuous.Derivative 默认 k=1, T=0.01 -> 一阶滤波微分:
    der(x) = (u - x)/T ,  y = (k/T)*(u - x)
"""
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

SIM = Path("/Users/yuchenxu/Desktop/MWorksUAVDelivery/MWorksUAVDelivery_Client_Final/03_Simulation_Results")
DEG = np.pi / 180.0


def filtered_derivative(u, t, T=0.01):
    """复刻 Modelica.Blocks.Continuous.Derivative (k=1, T)。
    状态 x: der(x) = (u - x)/T,  输出 y = (u - x)/T
    用精确指数积分（分段线性输入）离散，避免 T=0.01 下的显式积分发散。"""
    x = np.zeros_like(u)
    y = np.zeros_like(u)
    # Modelica 初始化: 稳态 x = u[0]  -> y[0] = 0
    x[0] = u[0]
    y[0] = (u[0] - x[0]) / T
    for i in range(1, len(u)):
        dt = t[i] - t[i - 1]
        a = np.exp(-dt / T)
        # 输入在步内线性: u(s) = u0 + (u1-u0)*s/dt
        u0, u1 = u[i - 1], u[i]
        # x' = (u - x)/T 的解析解
        x[i] = a * x[i - 1] + (1 - a) * u0 + (u1 - u0) * (1 - T / dt * (1 - a))
        y[i] = (u[i] - x[i]) / T
    return y


def integrator(u, t):
    """Modelica.Blocks.Continuous.Integrator (k=1), 梯形积分。"""
    return np.concatenate([[0.0], np.cumsum(0.5 * (u[1:] + u[:-1]) * np.diff(t))])


def pid_out(e, t, KP, KI, KD):
    return KP * e + KI * integrator(e, t) + KD * filtered_derivative(e, t)


def hybrid_smc_out(e, t, KP, KI, KD, lam, kSwitch, boundary, uMax):
    de = filtered_derivative(e, t)
    s = de + lam * e
    raw = KP * e + KI * integrator(e, t) + KD * de + kSwitch * np.tanh(s / max(boundary, 1e-6))
    return np.clip(raw, -uMax, uMax)


def reconstruct_tilt_cmd(df, variant):
    """返回 (pitch_cmd, roll_cmd)，单位 rad。"""
    t = df["time"].to_numpy(float)
    ex = df["ref_x"].to_numpy(float) - df["pos_x"].to_numpy(float)
    ey = df["ref_y"].to_numpy(float) - df["pos_y"].to_numpy(float)

    if variant == "baseline":
        ox = pid_out(ex, t, 1.5, 0.0, 1.0)
        oy = pid_out(ey, t, 1.5, 0.0, 1.0)
        lim = 15.0 * DEG
    else:
        ox = hybrid_smc_out(ex, t, 2.0, 0.0, 1.35, 1.25, 0.1, 0.35, 2.1)
        oy = hybrid_smc_out(ey, t, 2.0, 0.0, 1.35, 1.25, 0.1, 0.35, 2.1)
        lim = 9.5 * DEG

    pitch_cmd = np.clip(0.1 * ox, -lim, lim)
    roll_cmd = np.clip(0.1 * oy, -lim, lim)
    return pitch_cmd, roll_cmd


def main():
    scenarios = ["climb", "spiral", "eight"]
    variants = ["baseline", "enhanced"]
    report = {}
    for sc in scenarios:
        for va in variants:
            f = SIM / f"{sc}_{va}.csv"
            df = pd.read_csv(f)
            t = df["time"].to_numpy(float)
            pitch_cmd, roll_cmd = reconstruct_tilt_cmd(df, va)
            pitch = df["pitch"].to_numpy(float)
            roll = df["roll"].to_numpy(float)

            key = f"{sc}_{va}"
            report[key] = {
                "n": int(len(t)),
                "dt_median_s": float(np.median(np.diff(t))),
                "pitch_cmd_max_deg": float(np.abs(pitch_cmd).max() / DEG),
                "pitch_meas_max_deg": float(np.abs(pitch).max() / DEG),
                "roll_cmd_max_deg": float(np.abs(roll_cmd).max() / DEG),
                "roll_meas_max_deg": float(np.abs(roll).max() / DEG),
                # 反馈信号是 roll_cmd + roll (gain13=-1)，所以 -roll 才与 roll_cmd 同号
                "corr_pitch": float(np.corrcoef(pitch_cmd, pitch)[0, 1]),
                "corr_roll_plain": float(np.corrcoef(roll_cmd, roll)[0, 1]),
                "corr_roll_neg": float(np.corrcoef(roll_cmd, -roll)[0, 1]),
                "overshoot_ratio_pitch": float(
                    np.abs(pitch).max() / max(np.abs(pitch_cmd).max(), 1e-9)
                ),
                "overshoot_ratio_roll": float(
                    np.abs(roll).max() / max(np.abs(roll_cmd).max(), 1e-9)
                ),
            }
            print(f"{key:18s} "
                  f"cmd_pitch_max={report[key]['pitch_cmd_max_deg']:6.2f}deg "
                  f"meas_pitch_max={report[key]['pitch_meas_max_deg']:6.2f}deg "
                  f"ratio={report[key]['overshoot_ratio_pitch']:5.2f} | "
                  f"corr_p={report[key]['corr_pitch']:+.3f} "
                  f"corr_r(+)={report[key]['corr_roll_plain']:+.3f} "
                  f"corr_r(-)={report[key]['corr_roll_neg']:+.3f}")

    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("ident_stage1.json")
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\n-> {out}")


if __name__ == "__main__":
    main()
