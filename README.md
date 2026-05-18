# mapf_surveillance

SMT-based solver for the **Multi-Drone Persistent Surveillance Scheduling**
problem. Given a weighted graph of surveillance targets, a depot, a battery
budget, per-node latency deadlines, a planning horizon, and a fleet of drones,
the tool decides whether the fleet can patrol all targets without any drone
running out of battery or any node exceeding its visit deadline.

The encoding is built with [pySMT](https://pysmt.readthedocs.io) and solved
with [Z3](https://github.com/Z3Prover/z3) as the backend.

## Installation

```bash
pip install -r requirements.txt
```

If `pysmt` cannot find a Z3 binary, run:

```bash
pysmt-install --z3 --confirm-agreement
```

(Or rely on the `z3-solver` PyPI package, which ships Z3's Python bindings.)

## Input format

Scenario files are JSON. Example (`scenario.json`):

```json
{
  "nodes": [0, 1, 2, 3],
  "depot": 0,
  "edges": [[0, 1, 2], [1, 2, 1], [2, 3, 2], [3, 0, 1]],
  "latency": {"0": 999, "1": 4, "2": 5, "3": 4},
  "num_drones": 2,
  "battery": 8,
  "horizon": 12
}
```

* `edges` entries are `[node_a, node_b, travel_time]` and are undirected.
* `latency` keys are node ids (as strings); the depot uses the sentinel `999`
  to indicate "no latency constraint".
* `battery` is the per-drone budget that resets every time the drone is at the
  depot.
* `horizon` is the planning horizon `H` in unit timesteps. The solver
  produces paths of length `H + 1`.

## Usage

Solve a single scenario:

```bash
python solver.py --input scenario.json --output result.json --timeout 300
```

Override the horizon from the command line:

```bash
python solver.py --input scenario.json --horizon 15 --timeout 300
```

Run the full benchmark on every file in `instances/`:

```bash
python solver.py --benchmark --output benchmark_results.csv
# or
python benchmark.py
```

Generate the named test instances:

```bash
python generate_instances.py
```

## Output format

A satisfiable run produces (truncated):

```json
{
  "status": "SAT",
  "horizon": 12,
  "solve_time_seconds": 0.42,
  "num_variables": 240,
  "num_clauses": 890,
  "paths": [
    {"drone": 0, "path": [0, 1, 2, 1, 0, ...], "battery": [8, 6, 5, 6, 8, ...]},
    {"drone": 1, "path": [0, 3, 0, 3, 0, ...], "battery": [8, 7, 8, 7, 8, ...]}
  ],
  "validation": "PASS"
}
```

`UNSAT` and `TIMEOUT` results have `paths: null` and no `validation` field.

## SMT encoding

Variables:

| Symbol | Type | Meaning |
|---|---|---|
| `at[i][t][v]` | Bool | drone *i* is at node *v* at timestep *t* |
| `battery[i][t]` | Int | remaining battery for drone *i* at timestep *t* |

Constraint groups (each lives in its own function in
[encoding.py](encoding.py)):

1. **Initial / terminal placement** — every drone starts and ends at the
   depot.
2. **Exactly one location** — `ExactlyOne(at[i][t][v] for v)` per (drone,
   timestep).
3. **Valid movement** — between consecutive timesteps the drone either stays
   or traverses an edge.
4. **Latency satisfaction** — for every non-depot node *v* and every window of
   `T_v` consecutive timesteps, at least one drone visits *v*.
5. **Battery drain** — battery drops by the edge weight on every move,
   unchanged when staying.
6. **Battery reset** — battery equals `B` whenever the drone is at the depot.
7. **Battery non-negative** — `battery[i][t] >= 0` always.

A unit timestep represents one discrete action by each drone (move along an
edge or wait). Edge weights are treated as **battery cost per traversal**;
the "travel time must match" requirement is therefore enforced through the
battery accounting.

## Formula size

For each solve we report:

| Metric | Formula |
|---|---|
| Boolean variables | `k * (H + 1) * n` |
| Integer variables | `k * (H + 1)` |
| Latency clauses | `n * H` (approx) |
| Movement clauses | `k * H * n * (n - 1)` worst case (one disallow per missing edge) |
| Battery clauses | `k * H` |

The exact per-group clause counts are printed to stdout and persisted into the
output JSON (`clause_counts_per_group`).

## Project structure

```
mapf_surveillance/
├── solver.py              # CLI entry point
├── encoding.py            # SMT constraint functions (one per group)
├── validator.py           # post-solve solution validator
├── generate_instances.py  # writes JSON instances under ./instances
├── benchmark.py           # runs every instance and writes CSV + JSON results
├── instances/             # generated scenario JSONs
└── results/               # per-instance solver outputs
```

## Notes

* If the graph is disconnected and reachable nodes cannot cover all latency
  deadlines, the solver returns `UNSAT`. A warning is printed when nodes are
  unreachable from the depot.
* Timeouts are enforced by Z3's internal timeout option, so they work on
  Windows as well as POSIX systems.
* Every constraint function has type hints and a docstring describing the
  approximate number of clauses it contributes.
