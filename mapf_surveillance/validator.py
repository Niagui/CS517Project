"""Solution validator for the Multi-Drone Persistent Surveillance problem.

Given an instance and a list of drone paths, checks that every constraint is
satisfied. Returns a dict with `pass: bool` and a list of human-readable
violations.
"""
from typing import Any, Dict, List, Tuple


def validate_solution(
    instance: Dict[str, Any], paths: List[Dict[str, Any]], horizon: int
) -> Dict[str, Any]:
    """Check the paths against the instance constraints.

    Returns {"pass": bool, "violations": List[str]}.
    """
    violations: List[str] = []

    depot = instance["depot"]
    edges = instance["edges"]
    latency = instance["latency"]
    num_drones = instance["num_drones"]
    B = instance["battery"]
    num_nodes = len(instance["nodes"])

    edge_w: Dict[Tuple[int, int], int] = {}
    for a, b, w in edges:
        edge_w[(a, b)] = w
        edge_w[(b, a)] = w

    if len(paths) != num_drones:
        violations.append(
            f"Expected {num_drones} drone paths, got {len(paths)}"
        )

    for p in paths:
        drone = p["drone"]
        path = p["path"]
        bat = p["battery"]

        if len(path) != horizon + 1:
            violations.append(
                f"Drone {drone}: path length {len(path)} != H+1 ({horizon + 1})"
            )
            continue
        if len(bat) != horizon + 1:
            violations.append(
                f"Drone {drone}: battery length {len(bat)} != H+1 ({horizon + 1})"
            )
            continue

        if path[0] != depot:
            violations.append(
                f"Drone {drone}: starts at {path[0]}, expected depot {depot}"
            )
        if path[-1] != depot:
            violations.append(
                f"Drone {drone}: ends at {path[-1]}, expected depot {depot}"
            )

        if bat[0] != B:
            violations.append(
                f"Drone {drone}: initial battery {bat[0]} != B ({B})"
            )

        for t in range(horizon):
            u, v = path[t], path[t + 1]

            if u == v:
                expected = B if v == depot else bat[t]
                if bat[t + 1] != expected:
                    violations.append(
                        f"Drone {drone} t={t}->{t + 1}: stayed at {u}, "
                        f"battery {bat[t + 1]} != expected {expected}"
                    )
            else:
                if (u, v) not in edge_w:
                    violations.append(
                        f"Drone {drone} t={t}->{t + 1}: invalid move "
                        f"{u}->{v} (no edge)"
                    )
                    continue
                w = edge_w[(u, v)]
                expected = B if v == depot else (bat[t] - w)
                if bat[t + 1] != expected:
                    violations.append(
                        f"Drone {drone} t={t}->{t + 1}: move {u}->{v} (w={w}), "
                        f"battery {bat[t + 1]} != expected {expected}"
                    )

            if bat[t + 1] < 0:
                violations.append(
                    f"Drone {drone} t={t + 1}: battery {bat[t + 1]} < 0"
                )

    for v in range(num_nodes):
        if v == depot:
            continue
        T_v = _lookup_latency(latency, v)
        if T_v <= 0 or T_v >= 999:
            continue
        if T_v > horizon:
            visited = any(
                p["path"][t] == v for p in paths for t in range(horizon + 1)
            )
            if not visited:
                violations.append(
                    f"Node {v}: never visited in horizon (T={T_v})"
                )
            continue
        for t_start in range(horizon - T_v + 2):
            t_end = min(t_start + T_v, horizon + 1)
            visited = any(
                p["path"][t] == v for p in paths for t in range(t_start, t_end)
            )
            if not visited:
                violations.append(
                    f"Node {v}: latency violation, no visit in "
                    f"[{t_start},{t_end - 1}] (T={T_v})"
                )
                break

    return {"pass": len(violations) == 0, "violations": violations}


def _lookup_latency(latency: Dict[Any, int], v: int) -> int:
    """Latency JSON keys are strings; tolerate either form."""
    if str(v) in latency:
        return int(latency[str(v)])
    if v in latency:
        return int(latency[v])
    return 999
