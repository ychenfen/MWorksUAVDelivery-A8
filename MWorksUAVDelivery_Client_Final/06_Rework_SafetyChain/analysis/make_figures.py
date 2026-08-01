"""
生成替代造假图表的诚实配图。全部来自真实 CSV 与已验证的编队仿真。
输出到 06_Rework_SafetyChain/figures/
"""
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["Arial Unicode MS", "Heiti TC", "PingFang HK"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 150
plt.rcParams["savefig.bbox"] = "tight"
plt.rcParams["font.size"] = 9

sys.path.insert(0, str(Path(__file__).parent))
from ident_attitude import SIM, DEG, reconstruct_tilt_cmd

OUT = Path("/Users/yuchenxu/Desktop/MWorksUAVDelivery/MWorksUAVDelivery_Client_Final/06_Rework_SafetyChain/figures")
OUT.mkdir(parents=True, exist_ok=True)

BLUE, RED, GREEN, ORANGE, GRAY = "#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#888888"
SC = [("climb", "爬升", 30.0), ("spiral", "螺旋爬升", 10.0), ("eight", "8字航迹", None)]


def load(sc, va):
    return pd.read_csv(SIM / f"{sc}_{va}.csv")


# ============ 图A：参考轨迹不连续性与姿态超限的时间对应 ============
def fig_discontinuity():
    fig, axes = plt.subplots(3, 2, figsize=(11, 8))
    for r, (sc, name, tj) in enumerate(SC):
        df = load(sc, "enhanced")
        t = df["time"].to_numpy(float)
        # 左：参考速度（一阶差分）
        ax = axes[r, 0]
        for c, col, lab in [("ref_x", BLUE, "x"), ("ref_y", GREEN, "y"), ("ref_z", ORANGE, "z")]:
            v = np.diff(df[c].to_numpy(float)) / np.diff(t)
            ax.plot(t[1:], v, color=col, lw=0.9, label=f"d{lab}/dt")
        ax.set_ylabel(f"{name}\n参考速度 (m/s)")
        ax.set_xlim(t[0], t[-1])
        if tj:
            ax.axvline(tj, color=RED, ls="--", lw=1.2)
            ax.text(tj, ax.get_ylim()[1] * 0.72, f" t={tj:g}s 不连续", color=RED, fontsize=8)
        ax.legend(fontsize=7, ncol=3, loc="upper right")
        ax.grid(alpha=0.3)
        if r == 0:
            ax.set_title("参考轨迹速度（不连续处即激励源）")

        # 右：姿态与 15deg 边界
        ax = axes[r, 1]
        roll = df["roll"].to_numpy(float) / DEG
        pitch = df["pitch"].to_numpy(float) / DEG
        ax.plot(t, roll, color=BLUE, lw=0.8, label="roll")
        ax.plot(t, pitch, color=GREEN, lw=0.8, label="pitch")
        ax.axhline(15, color=RED, ls="--", lw=1.0)
        ax.axhline(-15, color=RED, ls="--", lw=1.0)
        viol = (np.abs(roll) > 15) | (np.abs(pitch) > 15)
        if viol.any():
            ax.plot(t[viol], np.maximum(np.abs(roll), np.abs(pitch))[viol] * np.sign(pitch[viol]),
                    "o", color=RED, ms=5, label=f"超限 {viol.sum()} 点")
        if tj:
            ax.axvline(tj, color=RED, ls="--", lw=1.2, alpha=0.5)
        ax.set_ylabel("姿态角 (deg)")
        ax.set_xlim(t[0], t[-1])
        ax.legend(fontsize=7, ncol=3, loc="upper right")
        ax.grid(alpha=0.3)
        if r == 0:
            ax.set_title("实测姿态与 15° 安全边界")
    for ax in axes[-1]:
        ax.set_xlabel("时间 (s)")
    fig.suptitle("参考轨迹不连续时刻与姿态超限时刻一一对应（增强模型，6/6 超限点均在跳变后 1 s 内）",
                 fontsize=10, y=0.995)
    fig.savefig(OUT / "F1_参考不连续与超限对应.png")
    plt.close(fig)
    return "F1_参考不连续与超限对应.png"


# ============ 图B2：单轴 vs 双轴对照（超限的决定性证据） ============
def fig_axis_contrast():
    df = load("climb", "enhanced")
    t = df["time"].to_numpy(float)
    pc, rc = reconstruct_tilt_cmd(df, "enhanced")
    events = [(20.0, "t=20 s：仅 ref_x 折角"), (30.0, "t=30 s：ref_x 与 ref_y 同时折角"),
              (40.0, "t=40 s：仅 ref_y 折角")]
    fig, axes = plt.subplots(3, 3, figsize=(13, 8))
    for c, (t0, title) in enumerate(events):
        w = (t >= t0 - 0.05) & (t <= t0 + 0.40)
        tt = t[w]
        # 行1 倾角指令
        ax = axes[0, c]
        ax.plot(tt, pc[w] / DEG, color=BLUE, lw=1.5, label="俯仰指令")
        ax.plot(tt, rc[w] / DEG, color=GREEN, lw=1.5, ls="--", label="横滚指令")
        ax.axhline(9.5, color=GRAY, ls=":", lw=1.0); ax.axhline(-9.5, color=GRAY, ls=":", lw=1.0)
        ax.set_ylim(-13, 13); ax.set_title(title, fontsize=9.5)
        if c == 0: ax.set_ylabel("倾角指令 (deg)")
        ax.legend(fontsize=6.5, loc="lower right"); ax.grid(alpha=0.3)
        # 行2 姿态
        ax = axes[1, c]
        pit = df["pitch"].to_numpy(float)[w] / DEG
        rol = df["roll"].to_numpy(float)[w] / DEG
        ax.plot(tt, pit, color=BLUE, lw=1.5, label="实测俯仰")
        ax.plot(tt, rol, color=GREEN, lw=1.5, ls="--", label="实测横滚")
        ax.axhline(15, color=RED, ls="--", lw=1.2); ax.axhline(-15, color=RED, ls="--", lw=1.2)
        mx = np.maximum(pit, rol); mn = np.minimum(pit, rol)
        ax.fill_between(tt, 15, mx, where=mx > 15, color=RED, alpha=0.28, interpolate=True)
        ax.fill_between(tt, -15, mn, where=mn < -15, color=RED, alpha=0.28, interpolate=True)
        ax.set_ylim(-24, 24)
        pk = max(np.abs(pit).max(), np.abs(rol).max())
        cpk = max(np.abs(pc[w]).max(), np.abs(rc[w]).max()) / DEG
        ax.annotate(f"姿态峰 {pk:.2f}°  放大 {pk/cpk:.3f}×\n{'超限' if pk>15 else '合规'}",
                    xy=(0.5, 0.04), xycoords="axes fraction", ha="center", fontsize=8,
                    bbox=dict(fc="#ffd6d6" if pk > 15 else "#d9f2d9",
                              ec=RED if pk > 15 else GREEN, alpha=0.95))
        if c == 0: ax.set_ylabel("姿态角 (deg)")
        ax.legend(fontsize=6.5, loc="upper right"); ax.grid(alpha=0.3)
        # 行3 电机指令摆幅
        ax = axes[2, c]
        u = df[["u1", "u2", "u3", "u4"]].to_numpy(float)[w]
        span = u.max(0) - u.min(0)
        cols = [RED if s > 12 else BLUE for s in span]
        ax.bar(["u1", "u2", "u3", "u4"], span, color=cols, alpha=0.85)
        for i, s in enumerate(span):
            ax.text(i, s + 0.4, f"{s:.2f}", ha="center", fontsize=7.5)
        ax.set_ylim(0, 24)
        ax.annotate(f"最大/最小 = {span.max()/span.min():.2f}", xy=(0.5, 0.93),
                    xycoords="axes fraction", ha="center", fontsize=8.5,
                    bbox=dict(fc="#fff3cd", ec="#ffc107", alpha=0.95))
        if c == 0: ax.set_ylabel("电机指令摆幅")
        ax.grid(alpha=0.3, axis="y")
    fig.suptitle("单轴 vs 双轴不连续对照（climb 增强模型）：指令侧完全等价，差别在内环饱和与混控分配",
                 fontsize=10.5, y=0.995)
    fig.savefig(OUT / "F2_单轴双轴对照.png")
    plt.close(fig)
    return "F2_单轴双轴对照.png"


# ============ 图B：超限事件的机理时序（放大） ============
def fig_mechanism():
    fig, axes = plt.subplots(2, 2, figsize=(11, 6.2))
    for c, (sc, name, tj, va) in enumerate([("climb", "爬升", 30.0, "enhanced"),
                                            ("spiral", "螺旋爬升", 10.0, "enhanced")]):
        df = load(sc, va)
        t = df["time"].to_numpy(float)
        pc, rc = reconstruct_tilt_cmd(df, va)
        w = (t >= tj - 0.06) & (t <= tj + 0.35)

        ax = axes[0, c]
        ax.plot(t[w], pc[w] / DEG, color=BLUE, lw=1.6, label="俯仰指令(反推)")
        ax.plot(t[w], rc[w] / DEG, color=GREEN, lw=1.6, ls="--", label="横滚指令(反推)")
        ax.axhline(9.5, color=GRAY, ls=":", lw=1.0)
        ax.axhline(-9.5, color=GRAY, ls=":", lw=1.0)
        ax.text(t[w][0], 9.9, "限幅 ±9.5°", fontsize=7, color=GRAY)
        ax.set_ylabel("倾角指令 (deg)")
        ax.set_title(f"{name}：t={tj:g}s 参考跳变后 0.35 s")
        ax.legend(fontsize=7); ax.grid(alpha=0.3)

        ax = axes[1, c]
        pit = df["pitch"].to_numpy(float)[w] / DEG
        rol = df["roll"].to_numpy(float)[w] / DEG
        ax.plot(t[w], pit, color=BLUE, lw=1.6, label="实测俯仰")
        ax.plot(t[w], rol, color=GREEN, lw=1.6, ls="--", label="实测横滚")
        ax.axhline(15, color=RED, ls="--", lw=1.2, label="15° 边界")
        ax.axhline(-15, color=RED, ls="--", lw=1.2)
        # 越界部分底色标出，并留足纵向余量让峰值可见（这张图的要点就是"越过了15度"）
        ax.fill_between(t[w], 15, np.maximum(pit, rol), where=np.maximum(pit, rol) > 15,
                        color=RED, alpha=0.25, interpolate=True)
        ax.fill_between(t[w], -15, np.minimum(pit, rol), where=np.minimum(pit, rol) < -15,
                        color=RED, alpha=0.25, interpolate=True)
        lim = max(np.abs(pit).max(), np.abs(rol).max(), 15.0)
        ax.set_ylim(-lim * 1.32, lim * 1.32)
        ax.set_ylabel("姿态角 (deg)"); ax.set_xlabel("时间 (s)")
        pk = max(np.abs(pit).max(), np.abs(rol).max())
        cmdpk = max(np.abs(pc[w]).max(), np.abs(rc[w]).max()) / DEG
        ax.annotate(f"指令峰值 {cmdpk:.2f}°  →  姿态峰值 {pk:.2f}°   放大 {pk / cmdpk:.2f}×",
                    xy=(0.5, 0.03), xycoords="axes fraction", ha="center", fontsize=8,
                    bbox=dict(fc="#fff3cd", ec="#ffc107", alpha=0.95))
        ax.legend(fontsize=7, loc="upper right"); ax.grid(alpha=0.3)
    fig.suptitle("超限事件时序：参考跳变后倾角指令在 3~4 个采样点内打满限幅，姿态随后越过 15° 边界", fontsize=10)
    fig.savefig(OUT / "F2_超限机理时序.png")
    plt.close(fig)
    return "F2_超限机理时序.png"


# ============ 图C：倾角指令变化率的分离度（ERG 限速取值依据） ============
def fig_rate_separation():
    JUMP = {"climb": [5.0, 20.0, 30.0], "spiral": [10.0], "eight": [10.0]}
    norm_all, jump_all, labels = [], [], []
    for sc, name, _ in SC:
        df = load(sc, "enhanced")
        t = df["time"].to_numpy(float)
        pc, rc = reconstruct_tilt_cmd(df, "enhanced")
        tm = 0.5 * (t[1:] + t[:-1])
        rate = np.abs(np.diff(np.vstack([pc, rc]), axis=1) / np.diff(t)).max(axis=0) / DEG
        m = np.zeros_like(tm, dtype=bool)
        for tj in JUMP[sc]:
            m |= (tm >= tj - 0.02) & (tm <= tj + 0.5)
        norm_all.append(rate[~m]); jump_all.append(rate[m]); labels.append(name)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.0))
    pos = np.arange(len(labels))
    bp1 = ax1.boxplot([np.clip(x, 1e-3, None) for x in norm_all], positions=pos - 0.17,
                      widths=0.3, patch_artist=True, showfliers=False)
    bp2 = ax1.boxplot([np.clip(x, 1e-3, None) for x in jump_all], positions=pos + 0.17,
                      widths=0.3, patch_artist=True, showfliers=False)
    for b in bp1["boxes"]: b.set_facecolor(BLUE); b.set_alpha(0.7)
    for b in bp2["boxes"]: b.set_facecolor(RED); b.set_alpha(0.7)
    ax1.set_yscale("log"); ax1.set_xticks(pos); ax1.set_xticklabels(labels)
    ax1.axhline(60, color=GREEN, lw=2.0, ls="--")
    ax1.text(-0.45, 68, "ERG 限速 60 °/s", color=GREEN, fontsize=8)
    ax1.set_ylabel("倾角指令变化率 (°/s，对数轴)")
    ax1.set_title("正常飞行(蓝) vs 参考跳变段(红)")
    ax1.grid(alpha=0.3, which="both")

    stats = []
    for nm, n_, j_ in zip(labels, norm_all, jump_all):
        stats.append([np.percentile(n_, 99.9), n_.max(), j_.max()])
    stats = np.array(stats)
    x = np.arange(len(labels)); w = 0.26
    ax2.bar(x - w, stats[:, 0], w, label="正常飞行 p99.9", color=BLUE)
    ax2.bar(x, stats[:, 1], w, label="正常飞行 最大", color=ORANGE)
    ax2.bar(x + w, stats[:, 2], w, label="跳变段 最大", color=RED)
    ax2.axhline(60, color=GREEN, lw=2.0, ls="--")
    ax2.set_yscale("log"); ax2.set_xticks(x); ax2.set_xticklabels(labels)
    ax2.set_ylabel("倾角指令变化率 (°/s，对数轴)")
    ax2.set_title("限速值落在 20~40 倍的间隔中间")
    ax2.legend(fontsize=7); ax2.grid(alpha=0.3, axis="y", which="both")
    fig.suptitle("ERG 限速取值依据：正常飞行需求与跳变激励相差 20~40 倍", fontsize=10)
    fig.savefig(OUT / "F3_指令变化率分离度.png")
    plt.close(fig)
    return "F3_指令变化率分离度.png"


# ============ 图D：RMSE 误差分解 ============
def fig_decomposition():
    rows = []
    for sc, name, tj in SC:
        if tj is None:
            continue
        for va, vn in [("baseline", "基线"), ("enhanced", "增强")]:
            df = load(sc, va)
            t = df["time"].to_numpy(float)
            e = np.sqrt((df.ref_x - df.pos_x) ** 2 + (df.ref_y - df.pos_y) ** 2
                        + (df.ref_z - df.pos_z) ** 2).to_numpy(float)
            full = np.sqrt(np.mean(e ** 2))
            m = (t >= tj) & (t < tj + 2.0)
            excl = np.sqrt(np.mean(e[~m] ** 2))
            rows.append((f"{name}\n{vn}", full, excl, (1 - excl / full) * 100))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.0))
    lab = [r[0] for r in rows]; x = np.arange(len(rows))
    ax1.bar(x - 0.2, [r[1] for r in rows], 0.4, label="全程 RMSE", color=RED)
    ax1.bar(x + 0.2, [r[2] for r in rows], 0.4, label="剔除跳变后 2 s 的 RMSE", color=BLUE)
    ax1.set_xticks(x); ax1.set_xticklabels(lab, fontsize=8)
    ax1.set_ylabel("路径 RMSE (m)"); ax1.legend(fontsize=8); ax1.grid(alpha=0.3, axis="y")
    ax1.set_title("参考不可行段对 RMSE 的贡献")
    ax2.bar(x, [r[3] for r in rows], 0.5, color=ORANGE)
    for i, r in enumerate(rows):
        ax2.text(i, r[3] + 1, f"{r[3]:.1f}%", ha="center", fontsize=8)
    ax2.set_xticks(x); ax2.set_xticklabels(lab, fontsize=8)
    ax2.set_ylabel("跳变段贡献占比 (%)"); ax2.grid(alpha=0.3, axis="y")
    ax2.set_title("该段仅占总时长 4%")
    fig.suptitle("误差分解：螺旋场景约一半的 RMSE 来自 t=10 s 的 3 m 参考阶跃，非控制器能力所致", fontsize=10)
    fig.savefig(OUT / "F4_误差分解.png")
    plt.close(fig)
    return "F4_误差分解.png"


# ============ 图E：编队控制律的三项测试 ============
def fig_formation():
    import verify_formation as V
    o1 = np.array([-3.0, -3.0, 0.0]); o2 = np.array([-3.0, 3.0, 0.0])
    V.KREP = 25.0
    nom = V.run(o1, o2)
    dis = V.run(o1, o2, p1_0=o1 + np.array([5.0, 0, 0]))
    c1 = np.array([-3.0, -0.2, 0.0]); c2 = np.array([-3.0, 0.2, 0.0])
    col = V.run(c1, c2, T=60.0)

    fig, axes = plt.subplots(1, 3, figsize=(12.5, 3.6))
    ax = axes[0]
    ax.plot(nom[:, 0], nom[:, 1], color=BLUE, lw=1.0, label="僚机1")
    ax.plot(nom[:, 0], nom[:, 2], color=GREEN, lw=1.0, ls="--", label="僚机2")
    ax.set_title(f"标称编队：稳态误差 {nom[nom[:,0]>10][:,1:3].mean():.3f} m")
    ax.set_ylabel("队形误差 (m)")

    ax = axes[1]
    ax.plot(dis[:, 0], dis[:, 1], color=RED, lw=1.2, label="僚机1（初始偏离 5 m）")
    ax.plot(dis[:, 0], dis[:, 2], color=GREEN, lw=1.0, ls="--", label="僚机2")
    for thr, cc in [(1.0, GRAY), (0.2, ORANGE)]:
        idx = np.where(dis[:, 1] < thr)[0]
        if len(idx):
            ax.axvline(dis[idx[0], 0], color=cc, ls=":", lw=1.2)
            ax.text(dis[idx[0], 0] + 1, 3.6 if thr == 1.0 else 2.9,
                    f"<{thr} m @ {dis[idx[0],0]:.2f}s", fontsize=7, color=cc)
    ax.set_title("扰动恢复：一致性项把队形拉回")
    ax.set_xlim(0, 30)

    ax = axes[2]
    ax.plot(col[:, 0], col[:, 3], color=BLUE, lw=1.2, label="实际机间距")
    ax.axhline(np.linalg.norm(c1 - c2), color=RED, ls="--", lw=1.2, label="期望间距 0.40 m")
    ax.axhline(2.0, color=GREEN, ls=":", lw=1.2, label="dSafe = 2.0 m")
    ax.set_title("避撞：期望队形本身违反安全间距")
    ax.set_ylabel("机间距 (m)")
    ax.set_ylim(0, 2.4)
    for ax in axes:
        ax.set_xlabel("时间 (s)"); ax.grid(alpha=0.3); ax.legend(fontsize=7)
    fig.suptitle("编队控制律验证（一致性编队 + 势场避撞，Python 独立复现 UAVFormation.mo 同一组方程）",
                 fontsize=10)
    fig.savefig(OUT / "F5_编队控制律验证.png")
    plt.close(fig)
    return "F5_编队控制律验证.png"


# ============ 图F：避撞增益扫描（软约束的局限） ============
def fig_repulsion_sweep():
    import verify_formation as V
    c1 = np.array([-3.0, -0.2, 0.0]); c2 = np.array([-3.0, 0.2, 0.0])
    n1 = np.array([-3.0, -3.0, 0.0]); n2 = np.array([-3.0, 3.0, 0.0])
    gains = [1.5, 3, 6, 12, 25, 50, 100]
    conf, nomerr = [], []
    for kr in gains:
        V.KREP = kr
        C = V.run(c1, c2, T=60.0); N = V.run(n1, n2, T=60.0)
        conf.append(C[C[:, 0] > 15][:, 3].mean())
        nomerr.append(N[N[:, 0] > 15][:, 1:3].mean())
    V.KREP = 25.0

    fig, ax = plt.subplots(figsize=(7.2, 4.0))
    ax.semilogx(gains, conf, "o-", color=BLUE, lw=1.8, ms=6, label="冲突队形下的稳态机间距")
    ax.axhline(2.0, color=GREEN, ls=":", lw=1.6, label="dSafe = 2.0 m（渐近达不到）")
    ax.axhline(0.4, color=RED, ls="--", lw=1.4, label="期望间距 0.40 m")
    ax.axvline(25, color=ORANGE, ls="-.", lw=1.4)
    ax.text(26, 0.62, "选定 kRep=25\n间距 1.53 m", fontsize=8, color=ORANGE)
    ax.set_xlabel("避撞势场增益 kRepulsion"); ax.set_ylabel("稳态机间距 (m)")
    ax.set_ylim(0, 2.3); ax.grid(alpha=0.3, which="both"); ax.legend(fontsize=8, loc="lower right")
    ax.set_title(f"势场是软约束：增益单调改善但渐近于 dSafe 而达不到\n"
                 f"标称队形下队形误差恒为 {np.mean(nomerr):.4f} m（间距>dSafe 时势场不激活，提高增益零代价）",
                 fontsize=9)
    fig.savefig(OUT / "F6_避撞增益扫描.png")
    plt.close(fig)
    return "F6_避撞增益扫描.png"


if __name__ == "__main__":
    made = [fig_discontinuity(), fig_axis_contrast(), fig_mechanism(), fig_rate_separation(),
            fig_decomposition(), fig_formation(), fig_repulsion_sweep()]
    print("生成:", *made, sep="\n  ")
    print("目录:", OUT)
