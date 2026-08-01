"""
基础仿真结果导出脚本说明

1. 本脚本在 MWorks / Sysplorer 中批量运行基础任务模型，并导出统一格式的 CSV / JSON 结果。
2. 导出的结果文件同时作为后续 Syslab 离线统计分析、指标对比和自动报告的数据输入接口。
3. 本次交付另附 UAVSyslabReferenceShaper.mo 与 UAVSysblockCommandAxis.mo 两个独立平台样例，
   用于说明当前版本已经直接使用 Syslab / Sysblock 能力；本脚本仍主要负责基础数据导出，而不是完整实时联合闭环。
"""

import csv
import json
import math
import os
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
if os.path.basename(SCRIPT_DIR) in ("02_Model_Code", "02_模型与代码"):
    parent = os.path.dirname(SCRIPT_DIR)
    out_name = "03_Simulation_Results" if os.path.basename(SCRIPT_DIR) == "02_Model_Code" else "03_仿真结果"
    OUT_DIR = os.path.join(parent, out_name)
else:
    OUT_DIR = os.path.join(SCRIPT_DIR, "simulation_outputs")

MODELS = [
    ("climb_baseline", ("QuadrotorModel.Task.ClimbBaseline", "QuadrotorTask.ClimbBaseline"), "climb"),
    ("climb_enhanced", ("QuadrotorModel.Task.ClimbEnhanced", "QuadrotorTask.ClimbEnhanced"), "climb"),
    ("spiral_baseline", ("QuadrotorModel.Task.SpiralBaseline", "QuadrotorTask.SpiralBaseline"), "spiral"),
    ("spiral_enhanced", ("QuadrotorModel.Task.SpiralEnhanced", "QuadrotorTask.SpiralEnhanced"), "spiral"),
    ("eight_baseline", ("QuadrotorModel.Task.EightBaseline", "QuadrotorTask.EightBaseline"), "eight"),
    ("eight_enhanced", ("QuadrotorModel.Task.EightEnhanced", "QuadrotorTask.EightEnhanced"), "eight"),
]

VARS = [
    "time",
    "climbePath.position_command[1]",
    "climbePath.position_command[2]",
    "climbePath.position_command[3]",
    "sensors1_1.PosMea[1]",
    "sensors1_1.PosMea[2]",
    "sensors1_1.PosMea[3]",
    "sensors1_1.AngleMea[1]",
    "sensors1_1.AngleMea[2]",
    "sensors1_1.AngleMea[3]",
    "controller3_2.y",
    "controller3_2.y1",
    "controller3_2.y2",
    "controller3_2.y3",
]


def is_finite(value):
    try:
        return math.isfinite(float(value))
    except Exception:
        return False


def last_violation_time(times, errors, band):
    last = None
    for t, err in zip(times, errors):
        if abs(err) > band:
            last = t
    return last


def compute_metrics(rows, scenario):
    n = len(rows)
    if n == 0:
        return {}

    times = [r[0] for r in rows]
    ref = [[r[1], r[2], r[3]] for r in rows]
    pos = [[r[4], r[5], r[6]] for r in rows]
    angle = [[r[7], r[8], r[9]] for r in rows]
    control = [[r[10], r[11], r[12], r[13]] for r in rows]
    error = [[pos[i][j] - ref[i][j] for j in range(3)] for i in range(n)]
    error_norm = [math.sqrt(sum(ej * ej for ej in e)) for e in error]

    rmse = [
        math.sqrt(sum(error[i][j] ** 2 for i in range(n)) / n)
        for j in range(3)
    ]
    max_abs = [max(abs(error[i][j]) for i in range(n)) for j in range(3)]
    max_angle = [max(abs(a[j]) for a in angle) * 180.0 / math.pi for j in range(3)]
    max_angle_norm = max(
        math.sqrt(sum(a[j] * a[j] for j in range(3))) * 180.0 / math.pi
        for a in angle
    )

    energy = 0.0
    for i in range(1, n):
        dt = times[i] - times[i - 1]
        u2 = sum(v * v for v in control[i])
        u2_prev = sum(v * v for v in control[i - 1])
        energy += 0.5 * dt * (u2 + u2_prev)

    metrics = {
        "samples": n,
        "start_time_s": times[0],
        "stop_time_s": times[-1],
        "rmse_x_m": rmse[0],
        "rmse_y_m": rmse[1],
        "rmse_z_m": rmse[2],
        "path_rmse_m": math.sqrt(sum(e * e for e in error_norm) / n),
        "mean_path_error_m": sum(error_norm) / n,
        "max_path_error_m": max(error_norm),
        "max_abs_x_m": max_abs[0],
        "max_abs_y_m": max_abs[1],
        "max_abs_z_m": max_abs[2],
        "max_roll_deg": max_angle[0],
        "max_pitch_deg": max_angle[1],
        "max_yaw_deg": max_angle[2],
        "max_attitude_norm_deg": max_angle_norm,
        "control_energy_proxy": energy,
        "angle_limit_15deg_violations": sum(
            1 for a in angle if max(abs(a[0]), abs(a[1])) > 15.0 * math.pi / 180.0
        ),
        "motor_abs_gt_7_count": sum(
            1 for u in control if max(abs(v) for v in u) > 7.0
        ),
        "command_span_x_m": max(r[1] for r in rows) - min(r[1] for r in rows),
        "command_span_y_m": max(r[2] for r in rows) - min(r[2] for r in rows),
        "command_span_z_m": max(r[3] for r in rows) - min(r[3] for r in rows),
    }

    if scenario == "climb":
        final_ref_z = ref[-1][2]
        z_error = [p[2] - final_ref_z for p in pos]
        overshoot = max(0.0, max(p[2] - final_ref_z for p in pos))
        band = 0.02 * max(abs(final_ref_z), 1e-9)
        last = last_violation_time(times, z_error, band)
        metrics.update(
            {
                "final_ref_z_m": final_ref_z,
                "final_z_m": pos[-1][2],
                "final_z_error_m": pos[-1][2] - final_ref_z,
                "z_overshoot_pct": overshoot / max(abs(final_ref_z), 1e-9) * 100.0,
                "z_settling_time_2pct_s": None if last is None or last >= times[-1] else last,
                "z_2pct_band_m": band,
            }
        )

    return metrics


def export_current_result(key, scenario):
    values = ModelingPy.GetVarsValues(VARS)
    if not isinstance(values, list) or len(values) != len(VARS):
        raise RuntimeError("Unexpected GetVarsValues result")

    length = min(len(v) for v in values)
    rows = []
    for i in range(length):
        row = []
        ok = True
        for col in values:
            value = col[i]
            if not is_finite(value):
                ok = False
                break
            row.append(float(value))
        if ok:
            rows.append(row)

    csv_path = os.path.join(OUT_DIR, key + ".csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "time",
                "ref_x",
                "ref_y",
                "ref_z",
                "pos_x",
                "pos_y",
                "pos_z",
                "roll",
                "pitch",
                "yaw",
                "u1",
                "u2",
                "u3",
                "u4",
            ]
        )
        writer.writerows(rows)

    metrics = compute_metrics(rows, scenario)
    metrics["csv"] = csv_path
    return metrics


def resolve_model(candidates):
    for model in candidates:
        try:
            if ModelingPy.ClassExist(model):
                return model
        except Exception:
            pass
    return candidates[0]


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    summary = {"metrics": {}, "improvements": {}, "status": [], "variables": VARS}
    start_all = time.time()

    for key, model_candidates, scenario in MODELS:
        start = time.time()
        model = resolve_model(model_candidates)
        sim_ok = ModelingPy.SimulateModel(model, 0)
        metrics = export_current_result(key, scenario)
        metrics["model"] = model
        metrics["scenario"] = scenario
        metrics["simulate_ok"] = bool(sim_ok)
        metrics["elapsed_s"] = time.time() - start
        summary["metrics"][key] = metrics
        summary["status"].append(
            {
                "key": key,
                "ok": bool(sim_ok),
                "rows": metrics.get("samples", 0),
                "elapsed_s": metrics["elapsed_s"],
            }
        )

    for scenario in ["climb", "spiral", "eight"]:
        base = summary["metrics"][scenario + "_baseline"]
        enhanced = summary["metrics"][scenario + "_enhanced"]
        item = {}
        for metric in [
            "path_rmse_m",
            "mean_path_error_m",
            "max_path_error_m",
            "control_energy_proxy",
            "max_attitude_norm_deg",
            "angle_limit_15deg_violations",
            "motor_abs_gt_7_count",
        ]:
            b = base.get(metric)
            e = enhanced.get(metric)
            if isinstance(b, (int, float)) and abs(b) > 1e-12 and isinstance(e, (int, float)):
                item[metric + "_reduction_pct"] = (b - e) / abs(b) * 100.0
        if scenario == "climb":
            b = base.get("z_overshoot_pct")
            e = enhanced.get("z_overshoot_pct")
            if isinstance(b, (int, float)) and abs(b) > 1e-12 and isinstance(e, (int, float)):
                item["z_overshoot_pct_reduction_pct"] = (b - e) / abs(b) * 100.0
        summary["improvements"][scenario] = item

    summary["total_elapsed_s"] = time.time() - start_all
    json_path = os.path.join(OUT_DIR, "metrics_summary.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    return {
        "json": json_path,
        "status": summary["status"],
        "improvements": summary["improvements"],
        "total_elapsed_s": summary["total_elapsed_s"],
    }


RUN_SCRIPT_RESULT = main()
