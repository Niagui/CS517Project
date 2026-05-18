"""Benchmark runner: solve every instance and tabulate results.

Iterates over every ``*.json`` file in ``instances/``, runs the solver with a
fixed timeout (default 120 s), writes per-instance result JSONs to
``results/``, and aggregates a CSV plus a stdout summary.
"""
from __future__ import annotations

import csv
import json
import os
import sys
from typing import Any, Dict, List

from solver import load_instance, solve_instance

INSTANCES_DIR = "instances"
RESULTS_DIR = "results"


def _dominant_latency(latency: Dict[str, int]) -> int:
    """Return the most-common non-depot latency (proxy for T)."""
    vals = [v for v in latency.values() if v < 900]
    return max(vals) if vals else -1


def run_benchmark(
    output_csv: str = "benchmark_results.csv", timeout: float = 120.0
) -> None:
    """Solve every instance and write results to CSV."""
    if not os.path.isdir(INSTANCES_DIR):
        print(
            f"[ERROR] {INSTANCES_DIR}/ not found. "
            "Run `python generate_instances.py` first.",
            file=sys.stderr,
        )
        return
    os.makedirs(RESULTS_DIR, exist_ok=True)

    instance_files = sorted(
        f for f in os.listdir(INSTANCES_DIR) if f.endswith(".json")
    )
    rows: List[Dict[str, Any]] = []

    header = (
        f"{'Instance':<25} {'n':>3} {'k':>3} {'T':>3} {'B':>3} {'H':>3} "
        f"{'Status':<10} {'Time(s)':>9} {'Vars':>8} {'Cls':>8}"
    )
    print()
    print(header)
    print("-" * len(header))

    for fname in instance_files:
        path = os.path.join(INSTANCES_DIR, fname)
        instance = load_instance(path)
        n = len(instance["nodes"])
        k = instance["num_drones"]
        T = _dominant_latency(instance["latency"])
        B = instance["battery"]
        H = instance["horizon"]

        try:
            result = solve_instance(instance, H, timeout=timeout, verbose=False)
        except Exception as exc:
            result = {
                "status": f"ERROR: {exc}",
                "solve_time_seconds": 0.0,
                "num_variables": 0,
                "num_clauses": 0,
            }

        result_path = os.path.join(RESULTS_DIR, fname)
        with open(result_path, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2)

        row = {
            "instance": fname,
            "n": n,
            "k": k,
            "T": T,
            "B": B,
            "H": H,
            "status": result["status"],
            "solve_time_seconds": round(result["solve_time_seconds"], 3),
            "num_variables": result.get("num_variables", 0),
            "num_clauses": result.get("num_clauses", 0),
            "validation": result.get("validation", ""),
        }
        rows.append(row)
        print(
            f"{fname:<25} {n:>3} {k:>3} {T:>3} {B:>3} {H:>3} "
            f"{row['status']:<10} {row['solve_time_seconds']:>9.3f} "
            f"{row['num_variables']:>8} {row['num_clauses']:>8}"
        )

    with open(output_csv, "w", newline="", encoding="utf-8") as f:
        if rows:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

    print(f"\n[INFO] CSV written to {output_csv}")
    print(f"[INFO] Per-instance JSONs written to {RESULTS_DIR}/")

    print("\n=== Scaling summary ===")
    _summarise(rows, "scale_n_", "n")
    _summarise(rows, "scale_k_", "k")
    _summarise(rows, "scale_T_", "T")
    _summarise(rows, "scale_B_", "B")
    _summarise(rows, "scale_H_", "H")


def _summarise(rows: List[Dict[str, Any]], prefix: str, dim_key: str) -> None:
    """Print a one-line summary for a given scaling dimension."""
    subset = [r for r in rows if r["instance"].startswith(prefix)]
    if not subset:
        return
    bad = [r for r in subset if r["status"] in ("TIMEOUT", "UNKNOWN")]
    avg = sum(r["solve_time_seconds"] for r in subset) / len(subset)
    msg = (
        f"  {prefix:<10} {len(subset)} instances, "
        f"avg solve {avg:.2f}s, timeouts: {len(bad)}"
    )
    if bad:
        first = min(bad, key=lambda r: r["solve_time_seconds"])
        msg += f" (first at {first['instance']}, {dim_key}={first[dim_key]})"
    print(msg)


if __name__ == "__main__":
    run_benchmark()
