"""SMT encoding of the Multi-Drone Persistent Surveillance Scheduling problem.

Each constraint group is implemented as a separate function. Variables:
  - at[i][t][v]   Bool : drone i is at node v at timestep t
  - battery[i][t] Int  : remaining battery of drone i at timestep t

Time is discrete with unit steps. An edge with travel time w costs w battery
units to traverse (consumed in a single timestep). Staying at a node consumes
no battery, and arriving at the depot resets battery to B.
"""
from typing import Any, Dict, List, Tuple

from pysmt.fnode import FNode
from pysmt.shortcuts import (
    And,
    BOOL,
    Equals,
    ExactlyOne,
    GE,
    INT,
    Implies,
    Int,
    Minus,
    Not,
    Or,
    Symbol,
)


def create_variables(
    num_drones: int, horizon: int, num_nodes: int
) -> Tuple[List[List[List[FNode]]], List[List[FNode]]]:
    """Create the `at` Boolean variables and the `battery` Integer variables.

    Returns (at, battery) where:
      at[i][t][v]   - Boolean symbol for drone i at node v at time t
      battery[i][t] - Integer symbol for drone i battery at time t

    Total variables: k*(H+1)*n booleans + k*(H+1) integers.
    """
    at = [
        [
            [Symbol(f"at_{i}_{t}_{v}", BOOL) for v in range(num_nodes)]
            for t in range(horizon + 1)
        ]
        for i in range(num_drones)
    ]
    battery = [
        [Symbol(f"bat_{i}_{t}", INT) for t in range(horizon + 1)]
        for i in range(num_drones)
    ]
    return at, battery


def initial_placement(
    at: List[List[List[FNode]]],
    depot: int,
    num_drones: int,
    horizon: int,
) -> FNode:
    """All drones start at depot at t=0 and return to depot at t=H.

    Clauses: 2 * k (one start-at-depot and one end-at-depot per drone).
    """
    constraints: List[FNode] = []
    for i in range(num_drones):
        constraints.append(at[i][0][depot])
        constraints.append(at[i][horizon][depot])
    return And(constraints)


def exactly_one_location(
    at: List[List[List[FNode]]],
    num_drones: int,
    horizon: int,
    num_nodes: int,
) -> FNode:
    """Each drone occupies exactly one node at every timestep.

    Clauses: k * (H+1) ExactlyOne constraints.
    """
    constraints: List[FNode] = []
    for i in range(num_drones):
        for t in range(horizon + 1):
            constraints.append(ExactlyOne([at[i][t][v] for v in range(num_nodes)]))
    return And(constraints)


def valid_movement(
    at: List[List[List[FNode]]],
    edges: List[List[int]],
    num_drones: int,
    horizon: int,
    num_nodes: int,
) -> FNode:
    """Between consecutive timesteps a drone stays put or moves along an edge.

    For every (drone, t, u, v) with u != v and no edge (u,v), we forbid the
    transition. Edges are treated as undirected.

    Clauses: <= k * H * n * (n-1) (one disallow per invalid pair).
    """
    edge_set: set = set()
    for a, b, _ in edges:
        edge_set.add((a, b))
        edge_set.add((b, a))

    constraints: List[FNode] = []
    for i in range(num_drones):
        for t in range(horizon):
            for u in range(num_nodes):
                for v in range(num_nodes):
                    if u != v and (u, v) not in edge_set:
                        constraints.append(Not(And(at[i][t][u], at[i][t + 1][v])))
    return And(constraints)


def latency_satisfaction(
    at: List[List[List[FNode]]],
    latency: Dict[Any, int],
    num_drones: int,
    horizon: int,
    num_nodes: int,
    depot: int,
) -> FNode:
    """For every non-depot node v and every window of T_v consecutive
    timesteps, at least one drone visits v.

    If T_v > horizon we require only that the node is visited at least once
    over the entire planning horizon. The depot is exempt (its sentinel
    latency value is typically 999).

    Clauses: ~ n * H (one Or per node per sliding window).
    """
    constraints: List[FNode] = []
    for v in range(num_nodes):
        if v == depot:
            continue
        T_v = _lookup_latency(latency, v)
        if T_v <= 0:
            continue
        if T_v >= 999:
            continue
        if T_v > horizon:
            visits = [
                at[i][t][v] for i in range(num_drones) for t in range(horizon + 1)
            ]
            if visits:
                constraints.append(Or(visits))
            continue
        for t_start in range(horizon - T_v + 2):
            t_end = min(t_start + T_v, horizon + 1)
            window = [
                at[i][t_p][v]
                for i in range(num_drones)
                for t_p in range(t_start, t_end)
            ]
            if window:
                constraints.append(Or(window))
    return And(constraints)


def battery_drain(
    at: List[List[List[FNode]]],
    battery: List[List[FNode]],
    edges: List[List[int]],
    num_drones: int,
    horizon: int,
    num_nodes: int,
    depot: int,
) -> FNode:
    """Battery decreases by the edge's travel time when a drone moves, and
    is unchanged when the drone stays put. Arrivals at the depot are handled
    by `battery_reset` (the value there is forced to B regardless).

    Clauses: ~ k * H * (n + 2 * |E|).
    """
    edge_weight: Dict[Tuple[int, int], int] = {}
    for a, b, w in edges:
        edge_weight[(a, b)] = w
        edge_weight[(b, a)] = w

    constraints: List[FNode] = []
    for i in range(num_drones):
        for t in range(horizon):
            for u in range(num_nodes):
                if u == depot:
                    continue
                constraints.append(
                    Implies(
                        And(at[i][t][u], at[i][t + 1][u]),
                        Equals(battery[i][t + 1], battery[i][t]),
                    )
                )
            for (u, v), w in edge_weight.items():
                if v == depot:
                    continue
                constraints.append(
                    Implies(
                        And(at[i][t][u], at[i][t + 1][v]),
                        Equals(battery[i][t + 1], Minus(battery[i][t], Int(w))),
                    )
                )
    return And(constraints)


def battery_reset(
    at: List[List[List[FNode]]],
    battery: List[List[FNode]],
    num_drones: int,
    horizon: int,
    depot: int,
    B: int,
) -> FNode:
    """Whenever a drone is at the depot, its battery equals B.

    Clauses: k * (H+1).
    """
    constraints: List[FNode] = []
    for i in range(num_drones):
        for t in range(horizon + 1):
            constraints.append(Implies(at[i][t][depot], Equals(battery[i][t], Int(B))))
    return And(constraints)


def battery_nonneg(
    battery: List[List[FNode]], num_drones: int, horizon: int
) -> FNode:
    """Battery is non-negative at every timestep.

    Clauses: k * (H+1).
    """
    constraints: List[FNode] = []
    for i in range(num_drones):
        for t in range(horizon + 1):
            constraints.append(GE(battery[i][t], Int(0)))
    return And(constraints)


def _lookup_latency(latency: Dict[Any, int], v: int) -> int:
    """Latency dictionaries from JSON have string keys; tolerate both."""
    if str(v) in latency:
        return int(latency[str(v)])
    if v in latency:
        return int(latency[v])
    return 999


def count_clauses(formulas: Dict[str, FNode]) -> Dict[str, int]:
    """Count top-level conjuncts per constraint group."""
    counts: Dict[str, int] = {}
    for name, f in formulas.items():
        if f.is_and():
            counts[name] = len(f.args())
        elif f.is_true():
            counts[name] = 0
        else:
            counts[name] = 1
    return counts
