from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
SIM = ROOT / "simulation_outputs"


def main() -> None:
    data = json.loads((SIM / "metrics_summary.json").read_text(encoding="utf-8"))
    metrics = data["metrics"]
    for key in ("climb_enhanced", "spiral_enhanced", "eight_enhanced"):
        out = SIM / f"{key}_metrics.json"
        out.write_text(json.dumps(metrics[key], ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
