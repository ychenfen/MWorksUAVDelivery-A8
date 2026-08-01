"""
独立复现 UAVFormation.mo 的方程，验证编队律：收敛性、扰动恢复、避撞。
方程与 .mo 一一对应，参数完全一致。
"""
import numpy as np
import pandas as pd
from ident_attitude import SIM

# ---- 与 .mo 完全一致的参数 ----
KC, KREP, DSAFE, DMIN, CMDLIM = 0.6, 1.5, 2.0, 0.3, 8.0
WNXY, ZEXY, WNZ, ZEZ, VMAX = 8.0, 0.7, 1.765, 0.257, 6.0
WN = np.array([WNXY, WNXY, WNZ]); ZE = np.array([ZEXY, ZEXY, ZEZ])


def formation_law(pL, p1, p2, off1, off2):
    des1, des2 = pL + off1, pL + off2
    eL1, eL2, e12 = p1 - pL, p2 - pL, p1 - p2
    dL1 = np.sqrt(eL1 @ eL1 + 1e-12); dL2 = np.sqrt(eL2 @ eL2 + 1e-12)
    d12 = np.sqrt(e12 @ e12 + 1e-12)
    nL1, nL2, n12 = max(dL1, DMIN), max(dL2, DMIN), max(d12, DMIN)

    cons1 = KC * (((pL - p1) - (-off1)) + ((p2 - p1) - (off2 - off1)))
    cons2 = KC * (((pL - p2) - (-off2)) + ((p1 - p2) - (off1 - off2)))

    w = lambda n: KREP * (1.0/n - 1.0/DSAFE) / (n*n) if n < DSAFE else 0.0
    rep1 = w(nL1) * (eL1/nL1) + w(n12) * (e12/n12)
    rep2 = w(nL2) * (eL2/nL2) - w(n12) * (e12/n12)

    cmd1 = des1 + np.clip(cons1 + rep1, -CMDLIM, CMDLIM)
    cmd2 = des2 + np.clip(cons2 + rep2, -CMDLIM, CMDLIM)
    return (cmd1, cmd2, np.linalg.norm(p1-des1), np.linalg.norm(p2-des2),
            min(dL1, dL2, d12))


def run(off1, off2, p1_0=None, p2_0=None, T=120.0, dt=0.005):
    df = pd.read_csv(SIM / "eight_enhanced.csv")
    tt = df["time"].to_numpy(float)
    ref = df[["ref_x", "ref_y", "ref_z"]].to_numpy(float)
    n = int(T/dt)
    pL = np.zeros(3); vL = np.zeros(3)
    p1 = (off1 if p1_0 is None else p1_0).astype(float).copy(); v1 = np.zeros(3)
    p2 = (off2 if p2_0 is None else p2_0).astype(float).copy(); v2 = np.zeros(3)
    log = []
    for k in range(n):
        t = k*dt
        cmdL = np.array([np.interp(t, tt, ref[:, i]) for i in range(3)])
        c1, c2, fe1, fe2, sep = formation_law(pL, p1, p2, off1, off2)
        for (p, v, c) in ((pL, vL, cmdL), (p1, v1, c1), (p2, v2, c2)):
            a = WN**2 * (c - p) - 2*ZE*WN*v
            v += dt*a
            p += dt*np.clip(v, -VMAX, VMAX)
        log.append((t, fe1, fe2, sep, np.linalg.norm(cmdL - pL)))
    return np.array(log)


off1 = np.array([-3.0, -3.0, 0.0]); off2 = np.array([-3.0, 3.0, 0.0])

print("=" * 78)
print("测试1 标称编队（ThreeUAVFormation）")
print("=" * 78)
L = run(off1, off2)
st = L[L[:, 0] > 10]
print(f"  队形误差: 稳态均值 {st[:,1:3].mean():.4f} m   稳态最大 {st[:,1:3].max():.4f} m"
      f"   全程最大 {L[:,1:3].max():.4f} m")
print(f"  最小机间距: {L[:,3].min():.3f} m  (安全阈值 dSafe={DSAFE} m)")
print(f"  领航机航迹跟踪误差 稳态均值 {st[:,4].mean():.4f} m")

print("\n" + "=" * 78)
print("测试2 扰动恢复（僚机1 初始偏离 5 m）")
print("=" * 78)
D = run(off1, off2, p1_0=off1 + np.array([5.0, 0, 0]))
e1 = D[:, 1]
print(f"  初始队形误差 {e1[0]:.3f} m")
for thr in [1.0, 0.5, 0.2, 0.1]:
    idx = np.where(e1 < thr)[0]
    print(f"  收敛到 <{thr:4.1f} m 用时: " + (f"{D[idx[0],0]:6.2f} s" if len(idx) else "未达到"))
print(f"  稳态(t>20s)队形误差均值 {D[D[:,0]>20][:,1].mean():.4f} m  -> "
      f"{'收敛' if D[D[:,0]>20][:,1].mean() < 0.2 else '未收敛'}")

print("\n" + "=" * 78)
print("测试3 避撞（期望队形本身只隔 0.4 m，小于 dSafe=2 m）")
print("=" * 78)
o1 = np.array([-3.0, -0.2, 0.0]); o2 = np.array([-3.0, 0.2, 0.0])
C = run(o1, o2, T=60.0)
print(f"  期望机间距 {np.linalg.norm(o1-o2):.2f} m")
print(f"  实际最小机间距 {C[:,3].min():.3f} m   稳态(t>10s)平均 {C[C[:,0]>10][:,3].mean():.3f} m")
print(f"  代价：队形误差被势场撑大到 {C[C[:,0]>10][:,1:3].mean():.3f} m（预期，属冲突下的取舍）")
ok = C[:, 3].min() > np.linalg.norm(o1-o2)
print(f"  -> 势场{'确实撑开了间距' if ok else '未能撑开间距'}")

print("\n" + "=" * 78)
print("对照：原 formation_demo() 的做法（固定偏移+0.12s延迟，无控制律）")
print("=" * 78)
df = pd.read_csv(SIM / "eight_enhanced.csv")
t = df["time"].to_numpy(float); lead = df[["pos_x","pos_y","pos_z"]].to_numpy(float)
offs = np.array([[0,0,0],[-3,-3,0],[-3,3,0]], float)
act = np.zeros((len(t),3,3)); act[0] = lead[0]+offs
for i in range(1,len(t)):
    dt_ = max(t[i]-t[i-1],1e-4); al = min(1.0, dt_/0.35)
    act[i,0] = lead[i]
    for j in (1,2):
        tgt = lead[max(0,i-int(0.12/dt_))]+offs[j]
        act[i,j] = act[i-1,j]+al*(tgt-act[i-1,j])
err = np.linalg.norm(act-(lead[:,None,:]+offs[None]),axis=2)
print(f"  其『最大队形误差』{err.max():.4f} m —— 纯延迟几何偏差，与控制无关；")
print(f"  施加扰动后无任何恢复机制，机间距不受约束（无避撞项）。")
