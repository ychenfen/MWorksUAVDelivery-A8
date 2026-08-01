"""x/y 共享参数联合拟合（结构上二者对称），合理边界，避免撞界假象。"""
import numpy as np, pandas as pd
from scipy.optimize import minimize
from ident_attitude import SIM
from ident_position import sim2nd, fitpct

df = pd.read_csv(SIM / "eight_enhanced.csv")
t = df["time"].to_numpy(float)
RX, PX = df["ref_x"].to_numpy(float), df["pos_x"].to_numpy(float)
RY, PY = df["ref_y"].to_numpy(float), df["pos_y"].to_numpy(float)
RZ, PZ = df["ref_z"].to_numpy(float), df["pos_z"].to_numpy(float)

def joint_cost(p):
    wn, ze, td = np.exp(p)
    if not (0.3 < wn < 15 and 0.15 < ze < 3.0 and td < 0.5):
        return 1e9
    try:
        a = sim2nd(RX, t, wn, ze, td); b = sim2nd(RY, t, wn, ze, td)
    except Exception:
        return 1e9
    if not (np.all(np.isfinite(a)) and np.all(np.isfinite(b))):
        return 1e9
    return float(np.mean((a-PX)**2) + np.mean((b-PY)**2))

best, bp = 1e18, None
for w0 in [1, 2, 4, 8]:
    for z0 in [0.4, 0.7, 1.0]:
        r = minimize(joint_cost, np.log([w0, z0, 0.05]), method="Nelder-Mead",
                     options={"maxiter": 1500, "fatol": 1e-12, "xatol": 1e-7})
        if r.fun < best: best, bp = r.fun, r.x
wn, ze, td = np.exp(bp)
print(f"水平通道共享参数: wn={wn:.4f} rad/s  zeta={ze:.4f}  tau={td:.4f} s")

def zcost(p):
    w, z, d = np.exp(p)
    if not (0.2 < w < 10 and 0.15 < z < 3 and d < 0.5): return 1e9
    h = sim2nd(RZ, t, w, z, d)
    return 1e9 if not np.all(np.isfinite(h)) else float(np.mean((h-PZ)**2))
bz, bzp = 1e18, None
for w0 in [0.5, 1, 2, 4]:
    r = minimize(zcost, np.log([w0, 0.7, 0.05]), method="Nelder-Mead",
                 options={"maxiter": 1500, "fatol": 1e-12, "xatol": 1e-7})
    if r.fun < bz: bz, bzp = r.fun, r.x
wz, zz, dz = np.exp(bzp)
print(f"高度通道参数:     wn={wz:.4f} rad/s  zeta={zz:.4f}  tau={dz:.4f} s")

print("\n跨场景验证（纯仿真，只喂参考；spiral 的 x 因 3m 跳变必然偏低）")
print(f"{'场景':10s}{'x':>9}{'y':>9}{'z':>9}")
for sc in ["climb", "spiral", "eight"]:
    d = pd.read_csv(SIM / f"{sc}_enhanced.csv"); tt = d["time"].to_numpy(float)
    f = [fitpct(d["pos_x"].to_numpy(float), sim2nd(d["ref_x"].to_numpy(float), tt, wn, ze, td)),
         fitpct(d["pos_y"].to_numpy(float), sim2nd(d["ref_y"].to_numpy(float), tt, wn, ze, td)),
         fitpct(d["pos_z"].to_numpy(float), sim2nd(d["ref_z"].to_numpy(float), tt, wz, zz, dz))]
    print(f"{sc:10s}" + "".join(f"{x:9.2f}" for x in f))

print(f"\nModelica 参数:\n  wnXY={wn:.4f}; zetaXY={ze:.4f}; tauXY={td:.4f};"
      f"\n  wnZ={wz:.4f}; zetaZ={zz:.4f}; tauZ={dz:.4f};")
