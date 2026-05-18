"""Multi-Drone Persistent Surveillance SMT Solver.

CLI entry point. Builds the encoding from `encoding.py`, hands it to Z3 via
pySMT, then reconstructs and validates the schedule. Usage:

    python solver.py --input scenario.json --output result.json --timeout 300
    python solver.py --input scenario.json --horizon 15
    python solver.py --benchmark --output benchmark_results.csv
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import Any, Dict, List

from pysmt.exceptions import SolverReturnedUnknownResultError
from pysmt.shortcuts import And, Solver

from encoding import (
    battery_drain,
    battery_nonneg,
    battery_reset,
    count_clauses,
    create_variables,
    exactly_one_location,
    initial_placement,
    latency_satisfaction,
    valid_movement,
)
from validator import validate_solution


def load_instance(path: str) -> Dict[str, Any]:
    """Load a scenario JSON file from disk."""
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _check_graph_connectivity(instance: Dict[str, Any]) -> List[str]:
    """Warn if any node is unreachable from the depot."""
    nodes = instance["nodes"]
    depot = instance["depot"]
    adj: Dict[int, List[int]] = {v: [] for v in nodes}
    for a, b, _ in instance["edges"]:
        adj[a].append(b)
        adj[b].append(a)
    seen = {depot}
    stack = [depot]
    while stack:
        u = stack.pop()
        for v in adj.get(u, []):
            if v not in seen:
                seen.add(v)
                stack.append(v)
    return [str(v) for v in nodes if v not in seen]


def solve_instance(
    instance: Dict[str, Any],
    horizon: int,
    timeout: float = 300.0,
    verbose: bool = True,
) -> Dict[str, Any]:
    """Solve a single scenario and return a result dict.

    Result fields: status, horizon, solve_time_seconds, num_variables,
    num_clauses, paths, validation, etc.
    """
    num_nodes = len(instance["nodes"])
    depot = instance["depot"]
    edges = instance["edges"]
    latency = instance["latency"]
    num_drones = instance["num_drones"]
    B = instance["battery"]

    if verbose:
        print(
            f"[INFO] Instance: n={num_nodes}, k={num_drones}, "
            f"H={horizon}, B={B}, |E|={len(edges)}"
        )

    unreachable = _check_graph_connectivity(instance)
    if unreachable and verbose:
        print(
            f"[WARN] Nodes unreachable from depot {depot}: "
            f"{', '.join(unreachable)} (instance likely UNSAT)"
        )

    if verbose:
        print("[INFO] Creating SMT variables...")
    at, battery = create_variables(num_drones, horizon, num_nodes)

    formulas: Dict[str, Any] = {}
    if verbose:
        print("[INFO] Encoding initial/terminal placement (group 1)...")
    formulas["initial"] = initial_placement(at, depot, num_drones, horizon)
    if verbose:
        print("[INFO] Encoding exactly-one location (group 2)...")
    formulas["exactly_one"] = exactly_one_location(at, num_drones, horizon, num_nodes)
    if verbose:
        print("[INFO] Encoding valid movement (group 3)...")
    formulas["movement"] = valid_movement(at, edges, num_drones, horizon, num_nodes)
    if verbose:
        print("[INFO] Encoding latency satisfaction (group 4)...")
    formulas["latency"] = latency_satisfaction(
        at, latency, num_drones, horizon, num_nodes, depot
    )
    if verbose:
        print("[INFO] Encoding battery drain (group 5)...")
    formulas["battery_drain"] = battery_drain(
        at, battery, edges, num_drones, horizon, num_nodes, depot
    )
    if verbose:
        print("[INFO] Encoding battery reset (group 6)...")
    formulas["battery_reset"] = battery_reset(at, battery, num_drones, horizon, depot, B)
    if verbose:
        print("[INFO] Encoding battery non-negative (group 7)...")
    formulas["battery_nonneg"] = battery_nonneg(battery, num_drones, horizon)

    full_formula = And(list(formulas.values()))

    num_bool_vars = num_drones * (horizon + 1) * num_nodes
    num_int_vars = num_drones * (horizon + 1)
    clause_counts = count_clauses(formulas)
    total_clauses = sum(clause_counts.values())

    if verbose:
        print("[INFO] Formula size:")
        print(f"        boolean variables: {num_bool_vars}")
        print(f"        integer variables: {num_int_vars}")
        print(f"        total clauses:     {total_clauses}")
        for name, c in clause_counts.items():
            print(f"          {name:<16} {c}")

    result: Dict[str, Any] = {
        "status": "UNKNOWN",
        "horizon": horizon,
        "solve_time_seconds": 0.0,
        "num_variables": num_bool_vars + num_int_vars,
        "num_bool_variables": num_bool_vars,
        "num_int_variables": num_int_vars,
        "num_clauses": total_clauses,
        "clause_counts_per_group": clause_counts,
        "paths": None,
    }

    timeout_ms = max(1, int(timeout * 1000))
    if verbose:
        print(f"[INFO] Solving with Z3 (timeout {timeout:.1f}s)...")

    start = time.time()
    try:
        with Solver(name="z3", solver_options={"timeout": timeout_ms}) as solver:
            solver.add_assertion(full_formula)
            try:
                is_sat = solver.solve()
            except SolverReturnedUnknownResultError:
                is_sat = None
            elapsed = time.time() - start
            result["solve_time_seconds"] = round(elapsed, 4)

            if is_sat is None:
                result["status"] = "TIMEOUT"
            elif is_sat:
                result["status"] = "SAT"
                model = solver.get_model()
                paths: List[Dict[str, Any]] = []
                for i in range(num_drones):
                    path: List[int] = []
                    bat: List[int] = []
                    for t in range(horizon + 1):
                        loc = None
                        for v in range(num_nodes):
                            if model.get_py_value(at[i][t][v]):
                                loc = v
                                break
                        path.append(loc if loc is not None else -1)
                        bat.append(int(model.get_py_value(battery[i][t])))
                    paths.append({"drone": i, "path": path, "battery": bat})
                result["paths"] = paths

                validation = validate_solution(instance, paths, horizon)
                result["validation"] = "PASS" if validation["pass"] else "FAIL"
                result["validation_details"] = validation
            else:
                result["status"] = "UNSAT"
    except Exception as exc:  # pragma: no cover - defensive
        elapsed = time.time() - start
        result["solve_time_seconds"] = round(elapsed, 4)
        result["status"] = f"ERROR: {exc}"

    if verbose:
        print(
            f"[INFO] Finished: {result['status']} in "
            f"{result['solve_time_seconds']:.3f}s"
        )
        if result.get("validation") == "FAIL":
            print("[WARN] Validation failed:")
            for v in result["validation_details"]["violations"]:
                print(f"  - {v}")

    return result


def main() -> int:
    """Parse CLI args and dispatch."""
    parser = argparse.ArgumentParser(
        description=(
            "Multi-Drone Persistent Surveillance SMT Solver. "
            "Uses pySMT with the Z3 backend."
        )
    )
    parser.add_argument("--input", help="Input scenario JSON file")
    parser.add_argument(
        "--output", help="Output result JSON file (or CSV with --benchmark)"
    )
    parser.add_argument(
        "--horizon", type=int, help="Override planning horizon from the JSON"
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=300.0,
        help="Per-instance solver timeout in seconds (default 300)",
    )
    parser.add_argument(
        "--benchmark",
        action="store_true",
        help="Run the benchmark on every instance in ./instances",
    )
    parser.add_argument(
        "--quiet", action="store_true", help="Suppress progress output"
    )
    args = parser.parse_args()

    if args.benchmark:
        from benchmark import run_benchmark

        run_benchmark(
            output_csv=args.output or "benchmark_results.csv",
            timeout=args.timeout if args.timeout != 300.0 else 120.0,
        )
        return 0

    if not args.input:
        parser.error("--input is required (unless --benchmark)")

    if not os.path.isfile(args.input):
        print(f"[ERROR] Input file not found: {args.input}", file=sys.stderr)
        return 1

    instance = load_instance(args.input)
    horizon = args.horizon if args.horizon is not None else int(instance["horizon"])

    result = solve_instance(
        instance, horizon, timeout=args.timeout, verbose=not args.quiet
    )

    if args.output:
        out_dir = os.path.dirname(os.path.abspath(args.output))
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(args.output, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2)
        if not args.quiet:
            print(f"[INFO] Result written to {args.output}")
    else:
        print(json.dumps(result, indent=2))

    return 0


if __name__ == "__main__":
    sys.exit(main())
