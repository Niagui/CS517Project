# Multi-Drone Persistent Surveillance via SMT — Technical Report

This report documents the `mapf_surveillance` tool: a Python program that
decides feasibility of the **Multi-Drone Persistent Surveillance Scheduling
problem (MDPS)** by reducing it to a satisfiability query in the SMT theory
of linear integer arithmetic with Booleans (`QF_LIA`), discharged by Z3 via
[pySMT](https://pysmt.readthedocs.io).

The bulk of the document is devoted to a precise statement of the reduction
and to a rigorous proof of its correctness (Section 3). Sections 4–6 cover
the implementation, the CLI, and the empirical evaluation.

---

## 1. Problem definition (MDPS)

**Input.** A tuple `I = (G, d, T, k, B, H)` where

- `G = (V, E, w)` is a finite undirected graph with edge-weight function
  `w : E → ℤ_{>0}` (the *travel time* / battery cost of an edge);
- `d ∈ V` is the **depot**;
- `T : V → ℤ_{>0} ∪ {∞}` is a per-node **latency deadline**; `T(d) = ∞`,
  and in practice `∞` is represented by the sentinel `999`;
- `k ∈ ℤ_{>0}` is the number of drones;
- `B ∈ ℤ_{>0}` is the battery budget;
- `H ∈ ℤ_{>0}` is the planning horizon (in unit timesteps).

Let `n = |V|`, and let timesteps range over `{0, 1, …, H}`.

**Output.** "SAT" together with a schedule, or "UNSAT".

**Schedule.** A *schedule* is a tuple `σ = (σ_0, …, σ_{k-1})` of maps
`σ_i : {0, …, H} → V`. A schedule is **valid** iff:

1. **(Endpoint)** `σ_i(0) = σ_i(H) = d` for every drone `i`.
2. **(Movement)** For every `i` and every `t ∈ {0, …, H-1}`,
   `σ_i(t+1) = σ_i(t)` or `(σ_i(t), σ_i(t+1)) ∈ E`.
3. **(Latency)** For every `v ∈ V \ {d}` and every window
   `[s, s + T_v − 1] ⊆ {0, …, H}` of size `T_v`,
   `∃ i, t ∈ [s, s + T_v − 1] : σ_i(t) = v`.
   If `T_v > H`, the constraint reduces to "`v` is visited at least once
   during the horizon".
4. **(Battery)** Define `b_i : {0, …, H} → ℤ` inductively by `b_i(0) = B`
   and, for `t ≥ 0`,

   ```
                    ⎧ B                            if σ_i(t+1) = d
   b_i(t + 1)  =    ⎨ b_i(t)                       if σ_i(t+1) = σ_i(t) ≠ d
                    ⎩ b_i(t) − w(σ_i(t), σ_i(t+1)) if σ_i(t+1) ≠ σ_i(t), ≠ d
   ```

   We require `b_i(t) ≥ 0` for every `i, t`.

The decision problem MDPS asks: does a valid schedule exist?

**Remark (modelling choice).** Time is discrete: every transition between
consecutive integer timesteps is one of (i) wait at the current node, or
(ii) traverse an incident edge. Edge weights are interpreted as the
*battery cost* incurred by traversing that edge in a single timestep; the
phrase "travel time must match" in the problem statement is therefore
realised by the battery accounting (rule 4), which forces the drop to equal
`w(u, v)`.

---

## 2. The target logic

The encoding produces a quantifier-free formula over

- a set of Boolean variables `at[i][t][v]` for
  `i ∈ {0,…,k-1}, t ∈ {0,…,H}, v ∈ V`;
- a set of Integer variables `bat[i][t]` for
  `i ∈ {0,…,k-1}, t ∈ {0,…,H}`.

All non-Boolean atoms are linear (in fact, `bat[i][t+1] = bat[i][t] − c`
for a constant `c`, `bat[i][t] = B`, and `bat[i][t] ≥ 0`), so the formula
lies in `QF_LIA + Bool`, which is decidable and supported natively by Z3.

The intended semantics is

```
at[i][t][v]  iff  σ_i(t) = v,
bat[i][t]    iff  b_i(t).
```

---

## 3. The reduction `MDPS → SMT`

For an instance `I = (G, d, T, k, B, H)` we construct a formula
`Φ(I) = Φ₁ ∧ Φ₂ ∧ Φ₃ ∧ Φ₄ ∧ Φ₅ ∧ Φ₆ ∧ Φ₇`.

Let `E↔ = { (u,v) : {u,v} ∈ E }` denote the directed view of `E` (edges are
undirected, so `(u,v) ∈ E↔ ⇔ (v,u) ∈ E↔`). Let
`w↔ : E↔ → ℤ_{>0}` be defined by `w↔(u,v) = w({u,v})`.

### Φ₁ — Endpoint placement

```
Φ₁ ≡ ⋀_{i=0}^{k-1} ( at[i][0][d] ∧ at[i][H][d] ).
```

### Φ₂ — Exactly one location per (drone, timestep)

```
Φ₂ ≡ ⋀_{i=0}^{k-1} ⋀_{t=0}^{H} ExactlyOne( at[i][t][v] : v ∈ V ).
```

`ExactlyOne(x₁, …, x_n)` is the standard pairwise encoding
`(⋁ x_j) ∧ ⋀_{j<j'} ¬(x_j ∧ x_{j'})`.

### Φ₃ — Movement along edges only

```
Φ₃ ≡ ⋀_{i, t<H} ⋀_{u ≠ v, (u,v) ∉ E↔} ¬( at[i][t][u] ∧ at[i][t+1][v] ).
```

Equivalently: for every `i` and `t`, `at[i][t][u] ∧ at[i][t+1][v]` implies
`u = v ∨ (u,v) ∈ E↔`.

### Φ₄ — Latency satisfaction

For each non-depot node `v` with `T_v ≤ H`:

```
   ⋀_{s=0}^{H − T_v + 1} ⋁_{i, t' ∈ [s, s + T_v − 1]} at[i][t'][v].
```

For each non-depot node `v` with `T_v > H` (and `T_v < ∞`):

```
   ⋁_{i, t ∈ [0, H]} at[i][t][v].
```

The conjunction over all such `v` defines Φ₄.

### Φ₅ — Battery drain

For every drone `i` and `t ∈ {0, …, H−1}`:

```
(a)  ⋀_{u ≠ d} ( at[i][t][u] ∧ at[i][t+1][u]   →   bat[i][t+1] = bat[i][t] )

(b)  ⋀_{(u,v) ∈ E↔, v ≠ d} ( at[i][t][u] ∧ at[i][t+1][v]
                              →  bat[i][t+1] = bat[i][t] − w↔(u,v) ).
```

Φ₅ is the conjunction of (a) and (b) over all `i, t`.

(*Why `v ≠ d` is excluded from both clauses: when the drone arrives at the
depot, Φ₆ overrides the value of `bat[i][t+1]` to `B`. Stating a separate
drain equation would over-constrain the model.*)

### Φ₆ — Battery reset at the depot

```
Φ₆ ≡ ⋀_{i, t} ( at[i][t][d]  →  bat[i][t] = B ).
```

### Φ₇ — Battery non-negative

```
Φ₇ ≡ ⋀_{i, t} bat[i][t] ≥ 0.
```

### 3.1 Decoding a model into a schedule

Given a model `M ⊨ Φ(I)`, define

```
σ_i(t) := the unique v ∈ V such that M(at[i][t][v]) = ⊤.
```

Uniqueness and existence follow from Φ₂. We call the resulting tuple
`σ(M) = (σ₀, …, σ_{k−1})` the **decoded schedule**.

### 3.2 Correctness theorem

**Theorem 1 (Soundness).** *If `Φ(I)` is satisfiable and `M ⊨ Φ(I)`, then
`σ(M)` is a valid schedule for `I`.*

**Theorem 2 (Completeness).** *If `I` admits a valid schedule, then `Φ(I)`
is satisfiable.*

Together, Theorems 1 and 2 prove that `Φ(·)` is a polynomial-time many-one
reduction from MDPS to SMT (`QF_LIA + Bool`).

---

### 3.3 Proofs

We write `M ⊨ φ` for "`M` satisfies `φ`" and treat all set-comprehension
indices over the same range conventions as in §3.

#### Proof of Theorem 1 (Soundness)

Fix a model `M ⊨ Φ(I)` and let `σ = σ(M)`. We check each clause of validity
(§1, items 1–4) in turn.

**(1) Endpoint.** Φ₁ ensures `M(at[i][0][d]) = ⊤` and
`M(at[i][H][d]) = ⊤`. By Φ₂, `σ_i(0)` and `σ_i(H)` are the unique nodes
with these flags set, so `σ_i(0) = σ_i(H) = d`. ✓

**(2) Movement.** Fix `i, t ∈ {0, …, H−1}`. Let `u = σ_i(t)`,
`v = σ_i(t+1)`. Then `M(at[i][t][u]) = M(at[i][t+1][v]) = ⊤`. Suppose for
contradiction that `u ≠ v` and `(u,v) ∉ E↔`. Then the corresponding clause
in Φ₃ would force `¬(at[i][t][u] ∧ at[i][t+1][v])`, contradicting our two
truths. Therefore `u = v` or `(u,v) ∈ E↔`. ✓

**(3) Latency.** Fix `v ∈ V \ {d}`.

  *Case `T_v ≤ H`.* Fix a window `[s, s + T_v − 1] ⊆ {0, …, H}`. The
  corresponding clause of Φ₄ is `⋁_{i, t' ∈ [s, s + T_v − 1]} at[i][t'][v]`.
  Since `M ⊨ Φ₄`, at least one disjunct is true, so there exists `(i, t')`
  with `σ_i(t') = v`. ✓

  *Case `H < T_v < ∞`.* By Φ₄ there exist `i, t` with
  `M(at[i][t][v]) = ⊤`, so `σ_i(t) = v`. ✓

  *Case `T_v = ∞`.* Φ₄ contains no clause for `v`; validity rule (3) is
  vacuous. ✓

**(4) Battery.** Let `β_i(t) := M(bat[i][t])`. We prove, by induction on
`t`, the joint invariant

```
( a )   β_i(t) = b_i(t),         where b_i is the value mandated by §1(4);
( b )   β_i(t) ≥ 0.
```

  *Base `t = 0`.* By Φ₁ and Φ₂, `σ_i(0) = d`, so `at[i][0][d]` is true and
  Φ₆ yields `β_i(0) = B = b_i(0)`. ✓ Non-negativity is Φ₇. ✓

  *Step.* Assume the invariant holds at `t`. Let `u = σ_i(t)`,
  `v = σ_i(t+1)`. By soundness of movement (above), `u = v` or
  `(u, v) ∈ E↔`. We split on `v`.

  - **If `v = d`.** Φ₆ forces `β_i(t+1) = B`. The rule for `b_i` in §1(4)
    also gives `b_i(t+1) = B`. Therefore `β_i(t+1) = b_i(t+1)`. ✓

  - **If `v ≠ d` and `u = v`.** The drone *stays* at `u ≠ d`. Φ₅(a)
    fires:`β_i(t+1) = β_i(t)`. The definition of `b_i` for the stationary
    case gives `b_i(t+1) = b_i(t)`. By the IH `β_i(t) = b_i(t)`, hence
    `β_i(t+1) = b_i(t+1)`. ✓

  - **If `v ≠ d` and `u ≠ v`.** Then `(u, v) ∈ E↔`. Φ₅(b) fires:
    `β_i(t+1) = β_i(t) − w↔(u,v)`. The definition of `b_i` for the
    edge-move case gives `b_i(t+1) = b_i(t) − w(u, v)`. By the IH and the
    fact that `w↔(u, v) = w({u, v})`, `β_i(t+1) = b_i(t+1)`. ✓

  Non-negativity at `t + 1` is the direct content of Φ₇.

  Thus both (a) and (b) hold at `t + 1`, completing the induction.

Since `β_i = b_i` and `β_i ≥ 0`, rule (4) of validity holds. ✓

All four validity conditions are satisfied, so `σ(M)` is a valid schedule.
∎

#### Proof of Theorem 2 (Completeness)

Let `σ = (σ_0, …, σ_{k-1})` be a valid schedule for `I`, and let
`b_i : {0,…,H} → ℤ_{≥0}` be the battery sequences from §1(4). Define an
assignment `M` to all SMT variables by:

```
M(at[i][t][v])  =  ⊤   iff   σ_i(t) = v,
M(bat[i][t])    =  b_i(t).
```

We verify `M ⊨ Φ_j` for every `j ∈ {1, …, 7}`.

**`M ⊨ Φ₁`.** By validity (1), `σ_i(0) = σ_i(H) = d`, so
`M(at[i][0][d]) = M(at[i][H][d]) = ⊤`. ✓

**`M ⊨ Φ₂`.** For each `(i, t)`, the function `σ_i` outputs exactly one
node, so exactly one of `{at[i][t][v]}_{v ∈ V}` is true under `M`. ✓

**`M ⊨ Φ₃`.** Pick any disallow clause
`¬(at[i][t][u] ∧ at[i][t+1][v])` with `u ≠ v, (u, v) ∉ E↔`. If the clause
were violated, both flags would be true under `M`, so `σ_i(t) = u` and
`σ_i(t+1) = v`. But validity (2) requires `σ_i(t+1) = σ_i(t)` or an edge;
neither is the case here, contradiction. Hence the clause holds. ✓

**`M ⊨ Φ₄`.** Fix any latency clause for a node `v ≠ d`.

  *Window clause (`T_v ≤ H`).* Validity (3) gives `i, t'` with
  `σ_i(t') = v` and `t' ∈ [s, s + T_v − 1]`, so `M(at[i][t'][v]) = ⊤`. ✓

  *Global clause (`H < T_v < ∞`).* Same argument with the trivial window
  `[0, H]`. ✓

**`M ⊨ Φ₅`.** Both (a) and (b) are implications. We need each implication
to hold under `M`. Fix `i, t`.

  Let `u = σ_i(t), v = σ_i(t+1)`. The antecedent of *exactly one*
  combination `(u', v')` is true under `M`: the one with `u' = u, v' = v`
  (by Φ₂-style uniqueness, already established for `M`). All other
  antecedents are false, and the implications hold vacuously.

  Consider the unique fired antecedent, with `v ≠ d`.

  - If `u = v ≠ d`, Φ₅(a) requires `bat[i][t+1] = bat[i][t]`. By validity
    (4), the stationary case sets `b_i(t+1) = b_i(t)`, so under `M` we
    have `M(bat[i][t+1]) = b_i(t+1) = b_i(t) = M(bat[i][t])`. ✓

  - If `u ≠ v, v ≠ d`, validity (2) gives `(u, v) ∈ E↔`, so the relevant
    Φ₅(b) clause is in `Φ`. It requires
    `bat[i][t+1] = bat[i][t] − w↔(u, v)`. By validity (4),
    `b_i(t+1) = b_i(t) − w(u, v) = b_i(t) − w↔(u, v)`. ✓

  All other antecedents in Φ₅(a)/(b) are false, hence vacuous.

  Therefore `M ⊨ Φ₅`. ✓

**`M ⊨ Φ₆`.** Pick a clause `at[i][t][d] → bat[i][t] = B`. If
`σ_i(t) = d`, validity (4) forces `b_i(t) = B` (since either `t = 0` or
the inductive case `σ_i(t) = d` triggered the reset). So
`M(bat[i][t]) = b_i(t) = B`, and the implication holds. If
`σ_i(t) ≠ d`, the antecedent is false. ✓

**`M ⊨ Φ₇`.** Validity (4) guarantees `b_i(t) ≥ 0` everywhere. ✓

All seven groups are satisfied; `M` is a model of `Φ(I)`. ∎

#### Polynomial size

Let `m = |E|`. The encoding produces

```
| at  vars |   = k(H+1)n
| bat vars |   = k(H+1)
| Φ₁     |   = 2k                                              clauses
| Φ₂     |   = k(H+1)  ExactlyOne  (≤ k(H+1)·(1 + n(n−1)/2))   clauses
| Φ₃     |   ≤ kH · n(n−1)                                    clauses
| Φ₄     |   ≤ n · (H+1)                                      clauses
| Φ₅     |   ≤ kH · (n − 1)  (stay) + kH · 2m  (move)         clauses
| Φ₆     |   = k(H+1)                                          clauses
| Φ₇     |   = k(H+1)                                          clauses
```

So `|Φ(I)| ∈ O(k · H · (n² + m))`, polynomial in `|I|`. The construction
itself runs in the same asymptotic time.

#### Corollary (Reduction)

The map `I ↦ Φ(I)` is a polynomial-time many-one reduction from MDPS to
SMT in the theory `QF_LIA + Bool`. Since Z3 is a sound and complete
decision procedure for this theory, the program is a sound and complete
decision procedure for MDPS (subject to memory and the configured solver
timeout).

---

## 4. Code organization

```
mapf_surveillance/
├── solver.py              CLI; loads JSON, calls encoding, runs Z3,
│                          decodes the model, validates, writes output.
├── encoding.py            One function per constraint group Φ_j.
├── validator.py           Independent re-check of every SAT result.
├── generate_instances.py  Materialises the named instances in §6.
├── benchmark.py           Runs every JSON in instances/, writes CSV.
├── requirements.txt
├── instances/             generated scenario JSONs
└── results/               solver outputs (one JSON per instance)
```

### 4.1 `encoding.py`

Eight functions:

- `create_variables(k, H, n)` allocates the two variable arrays.
- `initial_placement` encodes Φ₁.
- `exactly_one_location` encodes Φ₂.
- `valid_movement` encodes Φ₃.
- `latency_satisfaction` encodes Φ₄.
- `battery_drain` encodes Φ₅.
- `battery_reset` encodes Φ₆.
- `battery_nonneg` encodes Φ₇.

Each constraint-group function returns a single pySMT `FNode` of the form
`And([...])`. The number of conjuncts is the per-group "clause count"
reported by the solver.

### 4.2 `solver.py`

```
load_instance  →  create_variables  →  Φ₁…Φ₇  →  And(...)
              →  Solver(name='z3', timeout=...).add_assertion(Φ)
              →  solve()
              →  on SAT: decode at-vars into paths, decode bat-vars,
                          run validator, write JSON
              →  on UNSAT / TIMEOUT: write JSON with paths = null
```

`solver.py` also reports the per-group clause counts, the total variable
counts, and the wall-clock solve time. Timeout is enforced by Z3's
internal `timeout` solver option (milliseconds), which is portable to
Windows.

### 4.3 `validator.py`

The validator is independent of the encoder. Given a decoded schedule it
re-checks all four validity conditions in §1 directly on the paths. This
guards against:

- a bug in the encoder that admits a model violating some semantic rule,
- a downstream change to the JSON format that mangles a path.

If anything is amiss, the result JSON contains
`"validation": "FAIL"` together with a list of human-readable violations.

### 4.4 `benchmark.py`

Iterates every `*.json` in `instances/`, calls the solver with a 120-second
timeout, records `(instance, n, k, T, B, H, status, time, vars, clauses,
validation)` to `benchmark_results.csv`, persists per-instance result
JSONs under `results/`, and prints a scaling-dimension summary that
identifies which dimension produces the first TIMEOUT (if any).

---

## 5. How to run

### 5.1 Install

```bash
pip install -r requirements.txt
```

The `z3-solver` PyPI package ships Z3's Python bindings, so no separate
`pysmt-install --z3` step is necessary on most platforms.

### 5.2 Generate instances

```bash
python generate_instances.py
```

This populates `instances/` with the 22 named JSONs listed in §6.

### 5.3 Solve a single scenario

```bash
python solver.py --input instances/tiny_2n_1d.json \
                 --output results/tiny_2n_1d.json \
                 --timeout 30
```

Override the horizon:

```bash
python solver.py --input instances/scale_T_tight.json --horizon 20
```

Suppress progress output (still writes the JSON):

```bash
python solver.py --input scenario.json --output result.json --quiet
```

### 5.4 Run the full benchmark

```bash
python benchmark.py
# or, equivalently
python solver.py --benchmark --output benchmark_results.csv
```

### 5.5 Input file format

```json
{
  "nodes":      [0, 1, 2, 3],
  "depot":      0,
  "edges":      [[0,1,2], [1,2,1], [2,3,2], [3,0,1]],
  "latency":    {"0": 999, "1": 4, "2": 5, "3": 4},
  "num_drones": 2,
  "battery":    8,
  "horizon":    12
}
```

- `edges` entries are `[u, v, w]` and undirected.
- `latency` keys are stringified node IDs.
- `999` is the sentinel for "no latency constraint" (used for the depot).

### 5.6 Output file format

On `SAT`:

```json
{
  "status": "SAT",
  "horizon": 6,
  "solve_time_seconds": 0.058,
  "num_variables": 21,
  "num_bool_variables": 14,
  "num_int_variables":  7,
  "num_clauses": 36,
  "clause_counts_per_group": { "initial": 2, "exactly_one": 7, ... },
  "paths": [ { "drone": 0, "path": [...], "battery": [...] }, ... ],
  "validation": "PASS"
}
```

On `UNSAT` / `TIMEOUT`: `status` reflects the outcome and `paths` is
`null`.

---

## 6. Empirical evaluation

22 benchmark instances are generated by `generate_instances.py`. All use
cycle graphs with unit edge weights so that varying one parameter at a
time isolates its effect.

### 6.1 Instance families

| Family | Files | Fixed | Varies |
|---|---|---|---|
| correctness | `tiny_2n_1d`, `swap_3n_2d` | — | — |
| `scale_n_*` | n ∈ {3..8} | k=2, T=5, B=10, H=15 | `n` |
| `scale_k_*` | k ∈ {1..4} | n=5, T=5, B=10, H=15 | `k` |
| `scale_T_*` | tight/medium/loose | n=5, k=2, B=10, H=15 | `T ∈ {2,5,8}` |
| `scale_B_*` | tight/medium/loose | n=5, k=2, T=5, H=15 | `B ∈ {4,8,20}` |
| `scale_H_*` | H ∈ {10,15,20} | n=5, k=2, T=5, B=10 | `H` |

### 6.2 Results (full run, 120-second timeout per instance)

```
Instance                    n   k   T   B   H Status     Time(s)  Vars  Cls
scale_B_loose.json          5   2   5  20  15 SAT          0.043   192   808
scale_B_medium.json         5   2   5   8  15 SAT          0.025   192   808
scale_B_tight.json          5   2   5   4  15 SAT          0.026   192   808
scale_H_10.json             5   2   5  10  10 SAT          0.013   132   538
scale_H_15.json             5   2   5  10  15 SAT          0.023   192   808
scale_H_20.json             5   2   5  10  20 SAT          0.032   252  1078
scale_T_loose.json          5   2   8  10  15 SAT          0.027   192   796
scale_T_medium.json         5   2   5  10  15 SAT          0.026   192   808
scale_T_tight.json          5   2   2  10  15 UNSAT        0.017   192   820
scale_k_1.json              5   1   5  10  15 SAT          0.016    96   428
scale_k_2.json              5   2   5  10  15 SAT          0.021   192   808
scale_k_3.json              5   3   5  10  15 SAT          0.033   288  1188
scale_k_4.json              5   4   5  10  15 SAT          0.051   384  1568
scale_n_3.json              3   2   5  10  15 SAT          0.013   128   304
scale_n_4.json              4   2   5  10  15 SAT          0.017   160   526
scale_n_5.json              5   2   5  10  15 SAT          0.024   192   808
scale_n_6.json              6   2   5  10  15 SAT          0.032   224  1150
scale_n_7.json              7   2   5  10  15 SAT          0.036   256  1552
scale_n_8.json              8   2   5  10  15 UNSAT        0.043   288  2014
swap_3n_2d.json             3   2   3   6   6 SAT          0.007    56   128
tiny_2n_1d.json             2   1  10   5   6 SAT          0.003    21    36
```

All SAT cases validate `PASS`. The two UNSAT outcomes are both *expected*:

- `scale_T_tight` (T = 2): two drones cannot revisit four non-depot cycle
  nodes within every 2-timestep window.
- `scale_n_8` (n = 8, k = 2, T = 5): the cycle is too long for two drones
  to keep every node fresh within the 5-step deadline.

No instance times out at the chosen sizes; the bottleneck dimension for
future scaling is `n` (worst-case `Φ₃` is `kHn(n-1)`), in line with the
size analysis in §3.

---

## 7. Practical notes

- **Unreachable nodes.** Before solving, `solver.py` runs a BFS from the
  depot. Nodes unreachable from the depot are flagged with a `[WARN]`; the
  problem is then almost certainly UNSAT and the solver still returns the
  correct answer (no nondeterministic crash).
- **Cross-platform timeouts.** We use Z3's `timeout` solver option in
  milliseconds rather than POSIX signals, so the timeout works on Windows.
  A timeout is reported as `status: "TIMEOUT"`.
- **No iterative deepening.** The horizon is taken verbatim from the JSON
  (or the `--horizon` CLI flag). Wrapping a binary search over `H` around
  `solve_instance` is a natural next step but is intentionally outside the
  current scope.

---

## 8. Summary

`mapf_surveillance` is a literal implementation of the reduction
`MDPS → QF_LIA+Bool` defined in §3. Each of the seven constraint groups
lives in its own typed, documented function in `encoding.py`; the solver
glues them together with Z3 via pySMT; the validator independently
re-checks every SAT result; and the benchmark exercises the encoding
across five scaling dimensions. The reduction is polynomial-size and is
proved sound and complete in §3.3, so a satisfiability outcome from Z3
translates directly into a correct decision for the surveillance problem.
