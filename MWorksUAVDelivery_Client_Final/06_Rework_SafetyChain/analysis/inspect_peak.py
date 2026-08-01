"""看清楚 16.7deg 的姿态峰值到底发生在什么时刻、什么工况下。"""
import numpy as np
import pandas as pd
from ident_attitude import SIM, DEG, reconstruct_tilt_cmd

for sc, va in [("climb", "baseline"), ("climb", "enhanced"),
               ("spiral", "baseline"), ("spiral", "enhanced")]:
    df = pd.read_csv(SIM / f"{sc}_{va}.csv")
    t = df["time"].to_numpy(float)
    pc, rc = reconstruct_tilt_cmd(df, va)
    pitch = df["pitch"].to_numpy(float)
    roll = df["roll"].to_numpy(float)

    k = int(np.argmax(np.abs(pitch)))
    print(f"\n===== {sc}_{va} =====")
    print(f"pitch 峰值 {pitch[k]/DEG:+.2f}deg @ t={t[k]:.2f}s   该时刻指令={pc[k]/DEG:+.2f}deg")
    print(f"roll  峰值 {roll[int(np.argmax(np.abs(roll)))]/DEG:+.2f}deg "
          f"@ t={t[int(np.argmax(np.abs(roll)))]:.2f}s")
    # 超过 15deg 的时间段
    viol = np.abs(pitch) > 15 * DEG
    vr = np.abs(roll) > 15 * DEG
    if viol.any():
        print(f"pitch>15deg 的时间: {t[viol].min():.2f}~{t[viol].max():.2f}s  共 {viol.sum()} 个采样点")
    if vr.any():
        print(f"roll >15deg 的时间: {t[vr].min():.2f}~{t[vr].max():.2f}s  共 {vr.sum()} 个采样点")
    if not viol.any() and not vr.any():
        print("无 15deg 超限")

    # 前 3 秒的轨迹快照
    idx = [int(round(x / (t[1] - t[0]))) for x in [0.0, 0.1, 0.2, 0.3, 0.5, 0.8, 1.2, 2.0, 3.0, 5.0]]
    print(" t(s)   ref_z    pos_z   pitch_cmd  pitch    roll   ref_x   pos_x")
    for i in idx:
        if i < len(t):
            print(f"{t[i]:5.2f} {df['ref_z'][i]:8.3f} {df['pos_z'][i]:8.3f} "
                  f"{pc[i]/DEG:9.2f} {pitch[i]/DEG:7.2f} {roll[i]/DEG:7.2f} "
                  f"{df['ref_x'][i]:7.3f} {df['pos_x'][i]:7.3f}")
