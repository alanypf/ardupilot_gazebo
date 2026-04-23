#!/usr/bin/env python3
"""Convert a Flow5/APC PER3-style propeller performance file to a flat CSV
consumed by PropellerPerformancePlugin.

Output columns: rpm,v_ms,thrust_N,torque_Nm
Rows sorted by (rpm, v_ms).
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FLOW5_DIR = os.path.abspath(os.path.join(HERE, "..", "..", "Flow5-Performance"))
sys.path.insert(0, FLOW5_DIR)

from motor_prop_performance import load_propeller  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input", help="Path to PER3_*.txt Flow5 performance file")
    ap.add_argument("output", help="Path to write CSV")
    args = ap.parse_args()

    prop = load_propeller(args.input)
    points = sorted(prop.points, key=lambda p: (p.rpm, p.v_ms))

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["rpm", "v_ms", "thrust_N", "torque_Nm"])
        for p in points:
            w.writerow([f"{p.rpm:.1f}", f"{p.v_ms:.6f}",
                        f"{p.thrust_N:.6f}", f"{p.torque_Nm:.6f}"])

    print(f"Wrote {len(points)} rows to {args.output} "
          f"({prop.name}, {prop.diameter_in}in x {prop.pitch_in}in)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
