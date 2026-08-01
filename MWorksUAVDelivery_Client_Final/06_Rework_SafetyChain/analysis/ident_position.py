"""
辨识位置闭环 ref->pos 的二阶模型，供编队外环使用。
用连续二阶模型 wn, zeta 拟合，一场景拟合、其余场景验证。
"""
import numpy as np
import pandas as pd
from scipy.optimize import minimize
from scipy.integrate import solve_ivp
from ident_attitude import SIM

def sim2nd(ref, t, wn, zeta, tau_d=0.0):
    """二阶 + 可选一阶延迟，定步长 RK4。"""
    dt = t[1] - t[0]
    n = len(t)
    x = np.zeros(n); v = np.zeros(n); rf = np.zeros(n)
    for k in range(1, n):
        r = ref[k - 1]
        if tau_d > 1e-6:
            rf[k] = rf[k-1] + dt * (r - rf[k-1]) / tau_d
            r_eff = rf[k]
        else:
            r_eff = r
        a = wn**2 * (r_eff - x[k-1]) - 2*zeta*wn*v[k-1]
        v[k] = v[k-1] + dt*a
        x[k] = x[k-1] + dt*v[k]
    return x

def fit_axis(ref, pos, t):
    def cost(p):
        wn, zeta, td = np.exp(p[0]), np.exp(p[1]), np.exp(p[2])
        if not (0.05 < wn < 50 and 0.05 < zeta < 5 and td < 2):
            return 1e9
        try:
            xh = sim2nd(ref, t, wn, zeta, td)
        except Exception:
            return 1e9
        if not np.all(np.isfinite(xh)):
            return 1e9
        return float(np.mean((xh - pos)**2))
    best, bp = 1e18, None
    for w0 in [0.5, 1.0, 2.0, 4.0]:
        for z0 in [0.5, 0.8, 1.2]:
            r = minimize(cost, np.log([w0, z0, 0.05]), method="Nelder-Mead",
                         options={"maxiter": 800, "fatol": 1e-10, "xatol": 1e-6})
            if r.fun < best:
                best, bp = r.fun, r.x
    return np.exp(bp), best

def fitpct(y, yh):
    return 100*(1 - np.linalg.norm(y-yh)/np.linalg.norm(y-y.mean()))

print("辨识位置闭环 ref->pos 二阶模型（enhanced 控制器）")
print("=" * 84)
# 用 eight（无跳变、激励最丰富）做拟合
df = pd.read_csv(SIM / "eight_enhanced.csv")
t = df["time"].to_numpy(float)
params = {}
for ax, pc in [("ref_x","pos_x"), ("ref_y","pos_y"), ("ref_z","pos_z")]:
    p, mse = fit_axis(df[ax].to_numpy(float), df[pc].to_numpy(float), t)
    params[ax[-1]] = p
    print(f"  {ax[-1]} 轴: wn={p[0]:6.3f} rad/s  zeta={p[1]:5.3f}  tau_d={p[2]:5.3f}s   (拟合 MSE {mse:.3e})")

print("\n跨场景验证（纯仿真，只喂参考）:")
print(f"{'场景':16s}{'x':>9}{'y':>9}{'z':>9}")
for sc in ["climb", "spiral", "eight"]:
    df = pd.read_csv(SIM / f"{sc}_enhanced.csv")
    t = df["time"].to_numpy(float)
    fits = []
    for ax, pc in [("ref_x","pos_x"), ("ref_y","pos_y"), ("ref_z","pos_z")]:
        p = params[ax[-1]]
        xh = sim2nd(df[ax].to_numpy(float), t, *p)
        fits.append(fitpct(df[pc].to_numpy(float), xh))
    print(f"{sc:16s}" + "".join(f"{f:9.2f}" for f in fits))

print("\n用于 Modelica 编队模型的参数（取三轴折中）:")
wn = np.mean([params[a][0] for a in "xy"]); ze = np.mean([params[a][1] for a in "xy"])
td = np.mean([params[a][2] for a in "xy"])
print(f"  水平通道 wn={wn:.3f} rad/s, zeta={ze:.3f}, tau={td:.3f}s")
print(f"  高度通道 wn={params['z'][0]:.3f} rad/s, zeta={params['z'][1]:.3f}, tau={params['z'][2]:.3f}s")
