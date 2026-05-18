"""Convert a ``patrolling_sim`` ``.graph`` file into a scenario JSON.

The ``patrolling_sim`` project (https://github.com/davidbsp/patrolling_sim)
stores patrol maps as plain text. The file layout is::

    <num_nodes>
    <width>
    <height>
    <resolution>
    <origin_x>
    <origin_y>
    <blank>
    <node_id>
    <x>
    <y>
    <degree>
      <neighbour_id>
      <direction (N/S/E/W/NE/NW/SE/SW)>
      <distance>
      ...                              (repeated <degree> times)
    <blank>
    ...

This script reads such a file, extracts the (undirected) weighted graph,
and writes a JSON scenario compatible with ``solver.py``. CLI flags let you
choose the depot, number of drones, battery, latency, horizon, and an
optional integer divisor that rescales the edge weights (the raw distances
are typically too large for short horizons).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from typing import Any, Dict, List, Tuple


def parse_graph_file(path: str) -> Tuple[List[Tuple[int, int]], List[Tuple[int, int, int]]]:
    """Parse a patrolling_sim ``.graph`` file.

    Returns (positions, edges) where ``positions[i] = (x, y)`` and ``edges``
    contains undirected entries ``(u, v, w)`` with ``u < v``.
    """
    with open(path, "r", encoding="utf-8") as f:
        tokens = f.read().split()

    it = iter(tokens)

    def nxt() -> str:
        return next(it)

    num_nodes = int(nxt())
    _width = int(nxt())
    _height = int(nxt())
    _resolution = float(nxt())
    _origin_x = float(nxt())
    _origin_y = float(nxt())

    positions: List[Tuple[int, int]] = [(0, 0)] * num_nodes
    edge_dict: Dict[Tuple[int, int], int] = {}

    for _ in range(num_nodes):
        node_id = int(nxt())
        x = int(float(nxt()))
        y = int(float(nxt()))
        degree = int(nxt())
        positions[node_id] = (x, y)
        for _ in range(degree):
            neighbour = int(nxt())
            _direction = nxt()
            distance = int(float(nxt()))
            a, b = sorted((node_id, neighbour))
            edge_dict[(a, b)] = max(edge_dict.get((a, b), 0), distance)

    edges = [(a, b, w) for (a, b), w in sorted(edge_dict.items())]
    return positions, edges


def _connected_subset(
    num_nodes: int, edges: List[Tuple[int, int, int]], depot: int, max_nodes: int
) -> List[int]:
    """BFS from ``depot`` and return the first ``max_nodes`` reachable nodes."""
    adj: Dict[int, List[int]] = {v: [] for v in range(num_nodes)}
    for a, b, _ in edges:
        adj[a].append(b)
        adj[b].append(a)
    seen = [depot]
    frontier = [depot]
    visited = {depot}
    while frontier and len(seen) < max_nodes:
        nxt: List[int] = []
        for u in frontier:
            for v in adj[u]:
                if v not in visited:
                    visited.add(v)
                    seen.append(v)
                    nxt.append(v)
                    if len(seen) >= max_nodes:
                        break
            if len(seen) >= max_nodes:
                break
        frontier = nxt
    return seen


def build_instance(
    positions: List[Tuple[int, int]],
    edges: List[Tuple[int, int, int]],
    depot: int,
    num_drones: int,
    battery: int,
    latency: int,
    horizon: int,
    weight_divisor: int,
    max_nodes: int,
) -> Dict[str, Any]:
    """Assemble a JSON-ready instance dict from a parsed patrol graph."""
    n = len(positions)
    if depot < 0 or depot >= n:
        raise ValueError(f"depot {depot} out of range [0, {n - 1}]")

    if max_nodes > 0 and max_nodes < n:
        subset = _connected_subset(n, edges, depot, max_nodes)
        keep = set(subset)
        remap = {old: new for new, old in enumerate(subset)}
        new_depot = remap[depot]
        new_edges_raw = [(a, b, w) for a, b, w in edges if a in keep and b in keep]
        edges = [(remap[a], remap[b], w) for a, b, w in new_edges_raw]
        n = len(subset)
        depot = new_depot

    if weight_divisor <= 0:
        raise ValueError("weight_divisor must be positive")
    edges_scaled = [(a, b, max(1, int(math.ceil(w / weight_divisor)))) for a, b, w in edges]

    latency_map = {str(v): (999 if v == depot else latency) for v in range(n)}

    return {
        "nodes": list(range(n)),
        "depot": depot,
        "edges": [[a, b, w] for a, b, w in edges_scaled],
        "latency": latency_map,
        "num_drones": num_drones,
        "battery": battery,
        "horizon": horizon,
    }


def main() -> int:
    """CLI entry point: convert one ``.graph`` file into a scenario JSON."""
    parser = argparse.ArgumentParser(
        description=(
            "Convert a patrolling_sim .graph file into a mapf_surveillance "
            "scenario JSON."
        )
    )
    parser.add_argument("--input", required=True, help="Path to the .graph file")
    parser.add_argument("--output", required=True, help="Path to the output JSON")
    parser.add_argument("--depot", type=int, default=0, help="Depot node id (default 0)")
    parser.add_argument("--drones", type=int, default=2, help="Number of drones (default 2)")
    parser.add_argument("--battery", type=int, default=20, help="Battery budget B (default 20)")
    parser.add_argument(
        "--latency",
        type=int,
        default=15,
        help="Latency deadline T applied to every non-depot node (default 15)",
    )
    parser.add_argument("--horizon", type=int, default=20, help="Planning horizon H (default 20)")
    parser.add_argument(
        "--weight-divisor",
        type=int,
        default=10,
        help="Divide each raw edge distance by this integer (ceil), default 10",
    )
    parser.add_argument(
        "--max-nodes",
        type=int,
        default=0,
        help=(
            "If > 0, keep only the first N nodes reachable from the depot "
            "(BFS order). Useful for trimming large maps."
        ),
    )
    args = parser.parse_args()

    if not os.path.isfile(args.input):
        print(f"[ERROR] Input file not found: {args.input}", file=sys.stderr)
        return 1

    positions, edges = parse_graph_file(args.input)
    print(
        f"[INFO] Parsed {len(positions)} nodes and {len(edges)} undirected edges "
        f"from {args.input}"
    )

    instance = build_instance(
        positions=positions,
        edges=edges,
        depot=args.depot,
        num_drones=args.drones,
        battery=args.battery,
        latency=args.latency,
        horizon=args.horizon,
        weight_divisor=args.weight_divisor,
        max_nodes=args.max_nodes,
    )

    out_dir = os.path.dirname(os.path.abspath(args.output))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(instance, f, indent=2)

    n = len(instance["nodes"])
    m = len(instance["edges"])
    weights = [w for _, _, w in instance["edges"]]
    print(f"[INFO] Wrote {args.output}: n={n}, |E|={m}, weights in [{min(weights)}, {max(weights)}]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
