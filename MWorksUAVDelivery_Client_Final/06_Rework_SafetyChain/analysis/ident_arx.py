"""
第二步：辨识姿态内环闭环模型 cmd -> meas (ARX)，并用它设计/验证 ERG 限速。

做法：
  1. 用 ident_attitude.py 的拓扑反推倾角指令 u(t)，实测姿态 y(t)。
  2. 拟合 ARX(na, nb):  y[k] = -sum a_i y[k-i] + sum b_j u[k-j]
     在 climb_enhanced 上拟合，在 spiral/eight_enhanced 上做纯仿真验证（不喂真实 y）。
  3. 用辨识模型扫 ERG 限速率，找出能把峰值姿态压到 15deg 以下的最小限速。

已知偏差来源（如实记录，不藏）：
  - 闭环辨识：u 由实测位置反推，与 y 相关，参数存在闭环偏差。
  - 姿态回路与位置回路耦合，单入单出 ARX 只能近似。
  故本模型仅用于"定方案/定参数量级"，最终指标必须以 MWorks 重跑为准。
"""
import json
from pathlib import Path

import numpy as np
import pandas as pd

from ident_attitude import SIM, DEG, reconstruct_tilt_cmd

SC = ["climb", "spiral", "eight"]


def load(sc, va):
    df = pd.read_csv(SIM / f"{sc}_{va}.csv")
    t = df["time"].to_numpy(float)
    pc, rc = reconstruct_tilt_cmd(df, va)
    return t, pc, df["pitch"].to_numpy(float), rc, -df["roll"].to_numpy(float)


def fit_arx(u, y, na, nb, nk=1):
    N = len(y)
    p0 = max(na, nb + nk - 1)
    rows, rhs = [], []
    for k in range(p0, N):
        r = [-y[k - i] for i in range(1, na + 1)] + [u[k - nk - j + 1] for j in range(1, nb + 1)]
        rows.append(r)
        rhs.append(y[k])
    A = np.asarray(rows)
    b = np.asarray(rhs)
    theta, *_ = np.linalg.lstsq(A, b, rcond=None)
    return theta[:na], theta[na:]


def simulate_arx(a, b, u, y0=None, nk=1):
    """纯仿真：只用 u 和模型自身的历史输出，不喂真实 y。"""
    na, nb = len(a), len(b)
    N = len(u)
    y = np.zeros(N)
    p0 = max(na, nb + nk - 1)
    if y0 is not None:
        y[:p0] = y0[:p0]
    for k in range(p0, N):
        acc = 0.0
        for i in range(1, na + 1):
            acc -= a[i - 1] * y[k - i]
        for j in range(1, nb + 1):
            acc += b[j - 1] * u[k - nk - j + 1]
        y[k] = acc
    return y


def fit_pct(y, yhat):
    """Modelica/MATLAB 口径的拟合优度 (%)。"""
    return 100.0 * (1.0 - np.linalg.norm(y - yhat) / np.linalg.norm(y - y.mean()))


def rate_limit(u, t, rate):
    """ERGReferenceGovernor 的核心：一阶限速。tau 项在此简化为纯限速。"""
    out = np.zeros_like(u)
    out[0] = u[0]
    for k in range(1, len(u)):
        dt = t[k] - t[k - 1]
        d = np.clip(u[k] - out[k - 1], -rate * dt, rate * dt)
        out[k] = out[k - 1] + d
    return out


def main():
    # ---- 选阶：在 climb_enhanced pitch 上拟合，三场景验证 ----
    t_tr, u_tr, y_tr, _, _ = load("climb", "enhanced")
    best = None
    for na in (2, 3, 4):
        for nb in (1, 2, 3):
            a, b = fit_arx(u_tr, y_tr, na, nb)
            sims = {}
            for sc in SC:
                t, pc, pm, rc, rm = load(sc, "enhanced")
                sims[sc] = fit_pct(pm, simulate_arx(a, b, pc, pm))
            score = min(sims.values())
            if best is None or score > best["worst"]:
                best = {"na": na, "nb": nb, "a": a, "b": b, "fits": sims, "worst": score}
            print(f"ARX({na},{nb})  " + "  ".join(f"{k}={v:6.2f}%" for k, v in sims.items()))

    a, b = best["a"], best["b"]
    print(f"\n选定 ARX({best['na']},{best['nb']})，最差场景拟合优度 {best['worst']:.2f}%")
    print(f"  a = {np.array2string(a, precision=5)}")
    print(f"  b = {np.array2string(b, precision=6)}")

    # ---- 阶跃特性：模型预测的超调 ----
    ts = np.arange(0, 5, 0.01)
    step = np.full_like(ts, 9.5 * DEG)
    step[0] = 0.0
    ys = simulate_arx(a, b, step)
    print(f"  阶跃到 9.5deg 的模型预测峰值 = {ys.max()/DEG:.2f} deg (超调 {(ys.max()/(9.5*DEG)-1)*100:.1f}%)")

    # ---- 扫 ERG 限速率 ----
    print("\nERG 限速扫描（对 climb_enhanced 的倾角指令限速后，重跑辨识模型）:")
    rows = []
    for rate_deg in [None, 200, 120, 80, 60, 45, 35, 25, 18, 12, 8]:
        peaks = {}
        for sc in SC:
            t, pc, pm, rc, rm = load(sc, "enhanced")
            u = pc if rate_deg is None else rate_limit(pc, t, rate_deg * DEG)
            yy = simulate_arx(a, b, u, pm)
            peaks[sc] = np.abs(yy).max() / DEG
        tag = "无限速" if rate_deg is None else f"{rate_deg:4d} deg/s"
        flag = "OK " if max(peaks.values()) < 15.0 else "超限"
        rows.append({"rate_deg_s": rate_deg, **peaks})
        print(f"  {tag:>10s}  " + "  ".join(f"{k}={v:5.2f}deg" for k, v in peaks.items()) + f"   {flag}")

    Path("ident_arx.json").write_text(json.dumps({
        "order": {"na": best["na"], "nb": best["nb"]},
        "a": a.tolist(), "b": b.tolist(),
        "validation_fit_pct": best["fits"],
        "step_peak_deg": float(ys.max() / DEG),
        "erg_sweep": rows,
    }, indent=2, ensure_ascii=False), encoding="utf-8")
    print("\n-> ident_arx.json")


if __name__ == "__main__":
    main()
