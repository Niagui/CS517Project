"""Generate the named test instances described in the project brief.

Run with ``python generate_instances.py`` to populate the ``instances/``
folder. Each instance is a small connected graph; cycle graphs are used by
default because they keep parameter studies clean (every non-depot node is at
distance 1 from its two neighbours).
"""
from __future__ import annotations

import json
import os
from typing import Any, Dict

OUTPUT_DIR = "instances"


def make_cycle_instance(
    n: int, k: int, T: int, B: int, H: int, depot: int = 0
) -> Dict[str, Any]:
    """Build an n-node cycle graph with unit edge weights.

    Every non-depot node gets latency T; the depot gets the sentinel 999.
    """
    edges = [[i, (i + 1) % n, 1] for i in range(n)]
    latency = {str(v): (999 if v == depot else T) for v in range(n)}
    return {
        "nodes": list(range(n)),
        "depot": depot,
        "edges": edges,
        "latency": latency,
        "num_drones": k,
        "battery": B,
        "horizon": H,
    }


def write(name: str, instance: Dict[str, Any]) -> None:
    """Serialise an instance to ``instances/<name>``."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    path = os.path.join(OUTPUT_DIR, name)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(instance, f, indent=2)
    print(f"  wrote {path}")


def main() -> None:
    """Write every named instance referenced in agent.md."""
    print("Generating correctness tests...")
    tiny = {
        "nodes": [0, 1],
        "depot": 0,
        "edges": [[0, 1, 1]],
        "latency": {"0": 999, "1": 10},
        "num_drones": 1,
        "battery": 5,
        "horizon": 6,
    }
    write("tiny_2n_1d.json", tiny)

    swap = {
        "nodes": [0, 1, 2],
        "depot": 0,
        "edges": [[0, 1, 1], [1, 2, 1], [2, 0, 1]],
        "latency": {"0": 999, "1": 3, "2": 3},
        "num_drones": 2,
        "battery": 6,
        "horizon": 6,
    }
    write("swap_3n_2d.json", swap)

    print("Generating n-scaling instances (k=2, T=5, B=10, H=15)...")
    for n in range(3, 9):
        write(f"scale_n_{n}.json", make_cycle_instance(n, k=2, T=5, B=10, H=15))

    print("Generating k-scaling instances (n=5, T=5, B=10, H=15)...")
    for k in range(1, 5):
        write(f"scale_k_{k}.json", make_cycle_instance(5, k=k, T=5, B=10, H=15))

    print("Generating T-scaling instances (n=5, k=2, B=10, H=15)...")
    write("scale_T_tight.json", make_cycle_instance(5, k=2, T=2, B=10, H=15))
    write("scale_T_medium.json", make_cycle_instance(5, k=2, T=5, B=10, H=15))
    write("scale_T_loose.json", make_cycle_instance(5, k=2, T=8, B=10, H=15))

    print("Generating B-scaling instances (n=5, k=2, T=5, H=15)...")
    write("scale_B_tight.json", make_cycle_instance(5, k=2, T=5, B=4, H=15))
    write("scale_B_medium.json", make_cycle_instance(5, k=2, T=5, B=8, H=15))
    write("scale_B_loose.json", make_cycle_instance(5, k=2, T=5, B=20, H=15))

    print("Generating H-scaling instances (n=5, k=2, T=5, B=10)...")
    write("scale_H_10.json", make_cycle_instance(5, k=2, T=5, B=10, H=10))
    write("scale_H_15.json", make_cycle_instance(5, k=2, T=5, B=10, H=15))
    write("scale_H_20.json", make_cycle_instance(5, k=2, T=5, B=10, H=20))

    print("Done.")


if __name__ == "__main__":
    main()
