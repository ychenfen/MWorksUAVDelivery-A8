"""量化参考跳变对 RMSE / 姿态超限的贡献占比。"""
import numpy as np
import pandas as pd
from ident_attitude import SIM, DEG

def path_err(df):
    return np.sqrt((df.ref_x - df.pos_x) ** 2 + (df.ref_y - df.pos_y) ** 2 + (df.ref_z - df.pos_z) ** 2)

print("场景            全程RMSE   剔除跳变后2s   跳变段贡献   跳变段占时长")
print("-" * 78)
for sc, tj in [("climb", 30.0), ("spiral", 10.0), ("eight", None)]:
    for va in ["baseline", "enhanced"]:
        df = pd.read_csv(SIM / f"{sc}_{va}.csv")
        t = df["time"].to_numpy(float)
        e = path_err(df).to_numpy(float)
        full = np.sqrt(np.mean(e ** 2))
        if tj is None:
            print(f"{sc}_{va:9s} {full:8.4f}   (无跳变)")
            continue
        m = ~((t >= tj) & (t < tj + 2.0))
        excl = np.sqrt(np.mean(e[m] ** 2))
        contrib = (1 - excl / full) * 100
        print(f"{sc}_{va:9s} {full:8.4f}      {excl:8.4f}     {contrib:6.1f}%      "
              f"{2.0/t[-1]*100:5.1f}%")

print("\n\n姿态超限点是否全部落在跳变窗口内：")
for sc, tj in [("climb", 30.0), ("spiral", 10.0)]:
    for va in ["baseline", "enhanced"]:
        df = pd.read_csv(SIM / f"{sc}_{va}.csv")
        t = df["time"].to_numpy(float)
        v = (df.roll.abs() > 15 * DEG) | (df.pitch.abs() > 15 * DEG)
        n = int(v.sum())
        if n == 0:
            print(f"  {sc}_{va:9s}: 0 个超限点")
        else:
            tv = t[v.to_numpy()]
            inwin = int(((tv >= tj) & (tv < tj + 1.0)).sum())
            print(f"  {sc}_{va:9s}: {n} 个超限点, 落在 t={tj}~{tj+1}s 窗口内的 {inwin} 个 "
                  f"({inwin/n*100:.0f}%), 时刻 {np.array2string(tv, precision=2)}")

print("\n\n参考轨迹跳变清单：")
for sc in ["climb", "spiral", "eight"]:
    df = pd.read_csv(SIM / f"{sc}_enhanced.csv")
    t = df["time"].to_numpy(float)
    for ax in ["ref_x", "ref_y", "ref_z"]:
        d = np.abs(np.diff(df[ax].to_numpy(float)))
        big = np.where(d > 0.05)[0]
        # 速度折角：一阶差分的二阶差分
        dd = np.abs(np.diff(np.diff(df[ax].to_numpy(float))))
        corner = np.where(dd > 0.005)[0]
        s = f"{sc:7s} {ax}: 位置跳变 {len(big):2d} 处"
        if len(big):
            s += f" (最大 {d.max():.3f}m @ t={t[big[0]+1]:.2f}s)"
        s += f" | 速度折角 {len(corner):3d} 处"
        if len(corner):
            s += f" (最大 {dd.max():.4f} @ t={t[corner[0]+2]:.2f}s)"
        print(s)
