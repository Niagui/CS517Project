# Multi-Drone Persistent Surveillance via SMT — Technical Report

This report documents the `mapf_surveillance` tool: a Python program that
decides feasibility of the **Multi-Drone Persistent Surveillance Scheduling
problem (MDPS)** by reducing it to a satisfiability query in the SMT theory
of linear integer arithmetic with Booleans ($\mathsf{QF\_LIA}$), discharged
by Z3 via [pySMT](https://pysmt.readthedocs.io).

The bulk of the document is devoted to a precise statement of the reduction
and to a rigorous proof of its correctness (Section 3). Sections 4–6 cover
the implementation, the CLI, and the empirical evaluation.

---

## 1. Problem definition (MDPS)

**Input.** A tuple $I = (G, d, T, k, B, H)$ where

- $G = (V, E, w)$ is a finite undirected graph with edge-weight function
  $w : E \to \mathbb{Z}_{>0}$ (the *travel time* / battery cost of an edge);
- $d \in V$ is the **depot**;
- $T : V \to \mathbb{Z}_{>0} \cup \{\infty\}$ is a per-node **latency
  deadline**; $T(d) = \infty$ always, and any other node may also be set
  to $\infty$ to indicate "no latency constraint". In the JSON input,
  $\infty$ is represented by the sentinel value $999$;
- $k \in \mathbb{Z}_{>0}$ is the number of drones;
- $B \in \mathbb{Z}_{>0}$ is the battery budget;
- $H \in \mathbb{Z}_{>0}$ is the planning horizon (in unit timesteps).

Let $n = |V|$, and let timesteps range over $\{0, 1, \dots, H\}$.

**Output.** "SAT" together with a schedule, or "UNSAT".

**Schedule.** A *schedule* is a tuple $\sigma = (\sigma_0, \dots, \sigma_{k-1})$
of maps $\sigma_i : \{0, \dots, H\} \to V$. A schedule is **valid** iff:

1. **(Endpoint)** $\sigma_i(0) = \sigma_i(H) = d$ for every drone $i$.
2. **(Movement)** For every $i$ and every $t \in \{0, \dots, H-1\}$,
   $\sigma_i(t+1) = \sigma_i(t)$ or $(\sigma_i(t), \sigma_i(t+1)) \in E$.
3. **(Latency)** For every $v \in V \setminus \{d\}$ and every window
   $[s, s + T_v - 1] \subseteq \{0, \dots, H\}$ of size $T_v$,

$$\exists\, i,\; \exists\, t \in [s,\, s + T_v - 1] \;:\; \sigma_i(t) = v.$$

   If $T_v > H$, the constraint reduces to "$v$ is visited at least once
   during the horizon".
4. **(Battery)** Define $b_i : \{0, \dots, H\} \to \mathbb{Z}$ inductively
   by $b_i(0) = B$ and, for $t \geq 0$,

$$
b_i(t+1) \;=\;
\begin{cases}
B & \text{if } \sigma_i(t+1) = d, \\[4pt]
b_i(t) & \text{if } \sigma_i(t+1) = \sigma_i(t) \neq d, \\[4pt]
b_i(t) - w\bigl(\sigma_i(t),\, \sigma_i(t+1)\bigr) & \text{if } \sigma_i(t+1) \neq \sigma_i(t),\; \sigma_i(t+1) \neq d.
\end{cases}
$$

   We require $b_i(t) \geq 0$ for every $i, t$.

The decision problem MDPS asks: does a valid schedule exist?

**Remark (modelling choice).** Time is discrete: every transition between
consecutive integer timesteps is one of (i) wait at the current node, or
(ii) traverse an incident edge. Edge weights are interpreted as the
*battery cost* incurred by traversing that edge in a single timestep; the
phrase "travel time must match" in the problem statement is therefore
realised by the battery accounting (rule 4), which forces the drop to equal
$w(u, v)$.

---

## 2. The target logic

The encoding produces a quantifier-free formula over

- a set of Boolean variables $\mathrm{at}[i][t][v]$ for
  $i \in \{0, \dots, k-1\},\; t \in \{0, \dots, H\},\; v \in V$;
- a set of Integer variables $\mathrm{bat}[i][t]$ for
  $i \in \{0, \dots, k-1\},\; t \in \{0, \dots, H\}$.

All non-Boolean atoms are linear (in fact,
$\mathrm{bat}[i][t+1] = \mathrm{bat}[i][t] - c$ for a constant $c$,
$\mathrm{bat}[i][t] = B$, and $\mathrm{bat}[i][t] \geq 0$), so the formula
lies in $\mathsf{QF\_LIA} + \mathsf{Bool}$, which is decidable and supported
natively by Z3.

The intended semantics is

$$\mathrm{at}[i][t][v] \iff \sigma_i(t) = v, \qquad \mathrm{bat}[i][t] = b_i(t).$$

---

## 3. The reduction $\mathrm{MDPS} \to \mathrm{SMT}$

For an instance $I = (G, d, T, k, B, H)$ we construct a formula

$$\Phi(I) \;=\; \Phi_1 \wedge \Phi_2 \wedge \Phi_3 \wedge \Phi_4 \wedge \Phi_5 \wedge \Phi_6 \wedge \Phi_7.$$

Let $\overleftrightarrow{E} = \{(u, v) : \{u, v\} \in E\}$ denote the
directed view of $E$ (edges are undirected, so
$(u, v) \in \overleftrightarrow{E} \iff (v, u) \in \overleftrightarrow{E}$).
Let $\overleftrightarrow{w} : \overleftrightarrow{E} \to \mathbb{Z}_{>0}$ be
defined by $\overleftrightarrow{w}(u, v) = w(\{u, v\})$.

### $\Phi_1$ — Endpoint placement

$$\Phi_1 \;\equiv\; \bigwedge_{i=0}^{k-1} \Bigl(\mathrm{at}[i][0][d] \,\wedge\, \mathrm{at}[i][H][d]\Bigr).$$

### $\Phi_2$ — Exactly one location per (drone, timestep)

$$\Phi_2 \;\equiv\; \bigwedge_{i=0}^{k-1} \bigwedge_{t=0}^{H} \mathrm{ExactlyOne}\bigl(\mathrm{at}[i][t][v] : v \in V\bigr).$$

$\mathrm{ExactlyOne}(x_1, \dots, x_n)$ is the standard pairwise encoding

$$\Bigl(\bigvee_{j} x_j\Bigr) \;\wedge\; \bigwedge_{j < j'} \neg\bigl(x_j \wedge x_{j'}\bigr).$$

### $\Phi_3$ — Movement along edges only

$$\Phi_3 \;\equiv\; \bigwedge_{i,\, t < H} \;\; \bigwedge_{\substack{u \neq v \\ (u,v) \notin \overleftrightarrow{E}}} \neg\bigl(\mathrm{at}[i][t][u] \wedge \mathrm{at}[i][t+1][v]\bigr).$$

Equivalently: for every $i$ and $t$,
$\mathrm{at}[i][t][u] \wedge \mathrm{at}[i][t+1][v]$ implies
$u = v$ or $(u, v) \in \overleftrightarrow{E}$.

### $\Phi_4$ — Latency satisfaction

For each non-depot node $v$ with $T_v \leq H$:

$$\bigwedge_{s = 0}^{H - T_v + 1} \;\; \bigvee_{\substack{i \\ t' \in [s,\, s + T_v - 1]}} \mathrm{at}[i][t'][v].$$

For each non-depot node $v$ with $T_v > H$ (and $T_v < \infty$):

$$\bigvee_{\substack{i \\ t \in [0,\, H]}} \mathrm{at}[i][t][v].$$

The conjunction over all such $v$ defines $\Phi_4$.

### $\Phi_5$ — Battery drain

For every drone $i$ and $t \in \{0, \dots, H-1\}$:

$$
\text{(a)} \quad \bigwedge_{u \neq d} \Bigl(\mathrm{at}[i][t][u] \wedge \mathrm{at}[i][t+1][u] \;\to\; \mathrm{bat}[i][t+1] = \mathrm{bat}[i][t]\Bigr),
$$

$$
\text{(b)} \quad \bigwedge_{\substack{(u,v) \in \overleftrightarrow{E} \\ v \neq d}} \Bigl(\mathrm{at}[i][t][u] \wedge \mathrm{at}[i][t+1][v] \;\to\; \mathrm{bat}[i][t+1] = \mathrm{bat}[i][t] - \overleftrightarrow{w}(u, v)\Bigr).
$$

$\Phi_5$ is the conjunction of (a) and (b) over all $i, t$.

(*Why $v \neq d$ is excluded from both clauses: when the drone arrives at the
depot, $\Phi_6$ overrides the value of $\mathrm{bat}[i][t+1]$ to $B$.
Stating a separate drain equation would over-constrain the model.*)

### $\Phi_6$ — Battery reset at the depot

$$\Phi_6 \;\equiv\; \bigwedge_{i,\, t} \bigl(\mathrm{at}[i][t][d] \;\to\; \mathrm{bat}[i][t] = B\bigr).$$

### $\Phi_7$ — Battery non-negative

$$\Phi_7 \;\equiv\; \bigwedge_{i,\, t} \mathrm{bat}[i][t] \geq 0.$$

### 3.1 Decoding a model into a schedule

Given a model $M \models \Phi(I)$, define

$$\sigma_i(t) \;:=\; \text{the unique } v \in V \text{ such that } M\bigl(\mathrm{at}[i][t][v]\bigr) = \top.$$

Uniqueness and existence follow from $\Phi_2$. We call the resulting tuple
$\sigma(M) = (\sigma_0, \dots, \sigma_{k-1})$ the **decoded schedule**.

### 3.2 Correctness theorem

**Theorem 1 (Soundness).** *If $\Phi(I)$ is satisfiable and
$M \models \Phi(I)$, then $\sigma(M)$ is a valid schedule for $I$.*

**Theorem 2 (Completeness).** *If $I$ admits a valid schedule, then
$\Phi(I)$ is satisfiable.*

Together, Theorems 1 and 2 prove that $\Phi(\cdot)$ is a polynomial-time
many-one reduction from MDPS to SMT ($\mathsf{QF\_LIA} + \mathsf{Bool}$).

---

### 3.3 Proofs

We write $M \models \varphi$ for "$M$ satisfies $\varphi$" and treat all
set-comprehension indices over the same range conventions as in §3.

#### Proof of Theorem 1 (Soundness)

Fix a model $M \models \Phi(I)$ and let $\sigma = \sigma(M)$. We check each
clause of validity (§1, items 1–4) in turn.

**(1) Endpoint.** $\Phi_1$ ensures $M(\mathrm{at}[i][0][d]) = \top$ and
$M(\mathrm{at}[i][H][d]) = \top$. By $\Phi_2$, $\sigma_i(0)$ and
$\sigma_i(H)$ are the unique nodes with these flags set, so
$\sigma_i(0) = \sigma_i(H) = d$. $\checkmark$

**(2) Movement.** Fix $i, t \in \{0, \dots, H-1\}$. Let $u = \sigma_i(t)$,
$v = \sigma_i(t+1)$. Then $M(\mathrm{at}[i][t][u]) = M(\mathrm{at}[i][t+1][v]) = \top$.
Suppose for contradiction that $u \neq v$ and
$(u, v) \notin \overleftrightarrow{E}$. Then the corresponding clause in
$\Phi_3$ would force $\neg(\mathrm{at}[i][t][u] \wedge \mathrm{at}[i][t+1][v])$,
contradicting our two truths. Therefore $u = v$ or
$(u, v) \in \overleftrightarrow{E}$. $\checkmark$

**(3) Latency.** Fix $v \in V \setminus \{d\}$.

  _Case_ $T_v \leq H$. Fix a window $[s, s + T_v - 1] \subseteq \{0, \dots, H\}$.
  The corresponding clause of $\Phi_4$ is
  $\bigvee_{i,\, t' \in [s, s + T_v - 1]} \mathrm{at}[i][t'][v]$.
  Since $M \models \Phi_4$, at least one disjunct is true, so there exists
  $(i, t')$ with $\sigma_i(t') = v$. $\checkmark$

  _Case_ $H < T_v < \infty$. By $\Phi_4$ there exist $i, t$ with
  $M(\mathrm{at}[i][t][v]) = \top$, so $\sigma_i(t) = v$. $\checkmark$

  _Case_ $T_v = \infty$. $\Phi_4$ contains no clause for $v$; validity
  rule (3) is vacuous. $\checkmark$

**(4) Battery.** Let $\beta_i(t) := M(\mathrm{bat}[i][t])$. We prove, by
induction on $t$, the joint invariant

$$
\text{(a)}\quad \beta_i(t) = b_i(t), \qquad \text{(b)}\quad \beta_i(t) \geq 0,
$$

where $b_i$ is the value mandated by §1(4).

  _Base_ ($t = 0$). By $\Phi_1$ and $\Phi_2$, $\sigma_i(0) = d$, so
  $\mathrm{at}[i][0][d]$ is true and $\Phi_6$ yields $\beta_i(0) = B = b_i(0)$.
  $\checkmark$ Non-negativity is $\Phi_7$. $\checkmark$

  _Step._ Assume the invariant holds at $t$. Let $u = \sigma_i(t)$,
  $v = \sigma_i(t+1)$. By soundness of movement (above), $u = v$ or
  $(u, v) \in \overleftrightarrow{E}$. We split on $v$.

  - **If $v = d$.** $\Phi_6$ forces $\beta_i(t+1) = B$. The rule for $b_i$
    in §1(4) also gives $b_i(t+1) = B$. Therefore
    $\beta_i(t+1) = b_i(t+1)$. $\checkmark$

  - **If $v \neq d$ and $u = v$.** The drone *stays* at $u \neq d$.
    $\Phi_5\text{(a)}$ fires: $\beta_i(t+1) = \beta_i(t)$. The definition of
    $b_i$ for the stationary case gives $b_i(t+1) = b_i(t)$. By the IH
    $\beta_i(t) = b_i(t)$, hence $\beta_i(t+1) = b_i(t+1)$. $\checkmark$

  - **If $v \neq d$ and $u \neq v$.** Then $(u, v) \in \overleftrightarrow{E}$.
    $\Phi_5\text{(b)}$ fires:
    $\beta_i(t+1) = \beta_i(t) - \overleftrightarrow{w}(u, v)$. The definition
    of $b_i$ for the edge-move case gives $b_i(t+1) = b_i(t) - w(u, v)$. By
    the IH and the fact that $\overleftrightarrow{w}(u, v) = w(\{u, v\})$,
    $\beta_i(t+1) = b_i(t+1)$. $\checkmark$

  Non-negativity at $t + 1$ is the direct content of $\Phi_7$.

  Thus both (a) and (b) hold at $t + 1$, completing the induction.

Since $\beta_i = b_i$ and $\beta_i \geq 0$, rule (4) of validity holds.
$\checkmark$

All four validity conditions are satisfied, so $\sigma(M)$ is a valid
schedule. $\blacksquare$

#### Proof of Theorem 2 (Completeness)

Let $\sigma = (\sigma_0, \dots, \sigma_{k-1})$ be a valid schedule for $I$,
and let $b_i : \{0, \dots, H\} \to \mathbb{Z}_{\geq 0}$ be the battery
sequences from §1(4). Define an assignment $M$ to all SMT variables by:

$$M\bigl(\mathrm{at}[i][t][v]\bigr) = \top \iff \sigma_i(t) = v, \qquad M\bigl(\mathrm{bat}[i][t]\bigr) = b_i(t).$$

We verify $M \models \Phi_j$ for every $j \in \{1, \dots, 7\}$.

**$M \models \Phi_1$.** By validity (1), $\sigma_i(0) = \sigma_i(H) = d$,
so $M(\mathrm{at}[i][0][d]) = M(\mathrm{at}[i][H][d]) = \top$. $\checkmark$

**$M \models \Phi_2$.** For each $(i, t)$, the function $\sigma_i$ outputs
exactly one node, so exactly one of $\{\mathrm{at}[i][t][v]\}_{v \in V}$ is
true under $M$. $\checkmark$

**$M \models \Phi_3$.** Pick any disallow clause
$\neg(\mathrm{at}[i][t][u] \wedge \mathrm{at}[i][t+1][v])$ with $u \neq v$,
$(u, v) \notin \overleftrightarrow{E}$. If the clause were violated, both
flags would be true under $M$, so $\sigma_i(t) = u$ and
$\sigma_i(t+1) = v$. But validity (2) requires
$\sigma_i(t+1) = \sigma_i(t)$ or an edge; neither is the case here,
contradiction. Hence the clause holds. $\checkmark$

**$M \models \Phi_4$.** Fix any latency clause for a node $v \neq d$.

  _Window clause_ ($T_v \leq H$). Validity (3) gives $i, t'$ with
  $\sigma_i(t') = v$ and $t' \in [s, s + T_v - 1]$, so
  $M(\mathrm{at}[i][t'][v]) = \top$. $\checkmark$

  _Global clause_ ($H < T_v < \infty$). Same argument with the trivial
  window $[0, H]$. $\checkmark$

**$M \models \Phi_5$.** Both (a) and (b) are implications. We need each
implication to hold under $M$. Fix $i, t$.

  Let $u = \sigma_i(t),\; v = \sigma_i(t+1)$. The antecedent of *exactly
  one* combination $(u', v')$ is true under $M$: the one with
  $u' = u,\; v' = v$ (by $\Phi_2$-style uniqueness, already established for
  $M$). All other antecedents are false, and the implications hold
  vacuously.

  Consider the unique fired antecedent, with $v \neq d$.

  - If $u = v \neq d$, $\Phi_5\text{(a)}$ requires
    $\mathrm{bat}[i][t+1] = \mathrm{bat}[i][t]$. By validity (4), the
    stationary case sets $b_i(t+1) = b_i(t)$, so under $M$ we have
    $M(\mathrm{bat}[i][t+1]) = b_i(t+1) = b_i(t) = M(\mathrm{bat}[i][t])$.
    $\checkmark$

  - If $u \neq v,\; v \neq d$, validity (2) gives
    $(u, v) \in \overleftrightarrow{E}$, so the relevant $\Phi_5\text{(b)}$
    clause is in $\Phi$. It requires
    $\mathrm{bat}[i][t+1] = \mathrm{bat}[i][t] - \overleftrightarrow{w}(u, v)$.
    By validity (4),
    $b_i(t+1) = b_i(t) - w(u, v) = b_i(t) - \overleftrightarrow{w}(u, v)$.
    $\checkmark$

  All other antecedents in $\Phi_5\text{(a)}/\text{(b)}$ are false, hence
  vacuous.

  Therefore $M \models \Phi_5$. $\checkmark$

**$M \models \Phi_6$.** Pick a clause
$\mathrm{at}[i][t][d] \to \mathrm{bat}[i][t] = B$. If $\sigma_i(t) = d$,
validity (4) forces $b_i(t) = B$ (since either $t = 0$ or the inductive
case $\sigma_i(t) = d$ triggered the reset). So
$M(\mathrm{bat}[i][t]) = b_i(t) = B$, and the implication holds. If
$\sigma_i(t) \neq d$, the antecedent is false. $\checkmark$

**$M \models \Phi_7$.** Validity (4) guarantees $b_i(t) \geq 0$ everywhere.
$\checkmark$

All seven groups are satisfied; $M$ is a model of $\Phi(I)$. $\blacksquare$

#### Polynomial size

Let $m = |E|$. The encoding produces

| Component | Size |
|---|---|
| $\lvert \mathrm{at} \text{ vars} \rvert$ | $k(H+1)n$ |
| $\lvert \mathrm{bat} \text{ vars} \rvert$ | $k(H+1)$ |
| $\Phi_1$ | $2k$ clauses |
| $\Phi_2$ | $k(H+1)$ ExactlyOne $\leq k(H+1)\bigl(1 + \tfrac{n(n-1)}{2}\bigr)$ clauses |
| $\Phi_3$ | $\leq kH \cdot n(n-1)$ clauses |
| $\Phi_4$ | $\leq n \cdot (H+1)$ clauses |
| $\Phi_5$ | $\leq kH(n-1)$ (stay) $+\; kH \cdot 2m$ (move) clauses |
| $\Phi_6$ | $k(H+1)$ clauses |
| $\Phi_7$ | $k(H+1)$ clauses |

So $\lvert \Phi(I) \rvert \in \mathcal{O}\bigl(k \cdot H \cdot (n^2 + m)\bigr)$,
polynomial in $\lvert I \rvert$. The construction itself runs in the same
asymptotic time.

#### Corollary (Reduction)

The map $I \mapsto \Phi(I)$ is a polynomial-time many-one reduction from
MDPS to SMT in the theory $\mathsf{QF\_LIA} + \mathsf{Bool}$. Since Z3 is a
sound and complete decision procedure for this theory, the program is a
sound and complete decision procedure for MDPS (subject to memory and the
configured solver timeout).

---

## 4. Code organization

```
src/
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
- `initial_placement` encodes $\Phi_1$.
- `exactly_one_location` encodes $\Phi_2$.
- `valid_movement` encodes $\Phi_3$.
- `latency_satisfaction` encodes $\Phi_4$.
- `battery_drain` encodes $\Phi_5$.
- `battery_reset` encodes $\Phi_6$.
- `battery_nonneg` encodes $\Phi_7$.

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

This populates `instances/` with the 21 named JSONs listed in §6.

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

21 benchmark instances are generated by `generate_instances.py`. All use
cycle graphs with unit edge weights so that varying one parameter at a
time isolates its effect.

### 6.1 Instance families

| Family | Files | Fixed | Varies |
|---|---|---|---|
| correctness | `tiny_2n_1d`, `swap_3n_2d` | — | — |
| `scale_n_*` | $n \in \{3, \dots, 8\}$ | $k = 2,\, T = 5,\, B = 10,\, H = 15$ | $n$ |
| `scale_k_*` | $k \in \{1, \dots, 4\}$ | $n = 5,\, T = 5,\, B = 10,\, H = 15$ | $k$ |
| `scale_T_*` | tight/medium/loose | $n = 5,\, k = 2,\, B = 10,\, H = 15$ | $T \in \{2, 5, 8\}$ |
| `scale_B_*` | tight/medium/loose | $n = 5,\, k = 2,\, T = 5,\, H = 15$ | $B \in \{4, 8, 20\}$ |
| `scale_H_*` | $H \in \{10, 15, 20\}$ | $n = 5,\, k = 2,\, T = 5,\, B = 10$ | $H$ |

### 6.2 Results (full run, 120-second timeout per instance)

| Instance | $n$ | $k$ | $T$ | $B$ | $H$ | Status | Time (s) | Vars | Clauses |
|---|---:|---:|---:|---:|---:|:---:|---:|---:|---:|
| `scale_B_loose.json`   | 5 | 2 | 5 | 20 | 15 | SAT    | 0.043 | 192 |  808 |
| `scale_B_medium.json`  | 5 | 2 | 5 |  8 | 15 | SAT    | 0.025 | 192 |  808 |
| `scale_B_tight.json`   | 5 | 2 | 5 |  4 | 15 | SAT    | 0.026 | 192 |  808 |
| `scale_H_10.json`      | 5 | 2 | 5 | 10 | 10 | SAT    | 0.013 | 132 |  538 |
| `scale_H_15.json`      | 5 | 2 | 5 | 10 | 15 | SAT    | 0.023 | 192 |  808 |
| `scale_H_20.json`      | 5 | 2 | 5 | 10 | 20 | SAT    | 0.032 | 252 | 1078 |
| `scale_T_loose.json`   | 5 | 2 | 8 | 10 | 15 | SAT    | 0.027 | 192 |  796 |
| `scale_T_medium.json`  | 5 | 2 | 5 | 10 | 15 | SAT    | 0.026 | 192 |  808 |
| `scale_T_tight.json`   | 5 | 2 | 2 | 10 | 15 | UNSAT  | 0.017 | 192 |  820 |
| `scale_k_1.json`       | 5 | 1 | 5 | 10 | 15 | SAT    | 0.016 |  96 |  428 |
| `scale_k_2.json`       | 5 | 2 | 5 | 10 | 15 | SAT    | 0.021 | 192 |  808 |
| `scale_k_3.json`       | 5 | 3 | 5 | 10 | 15 | SAT    | 0.033 | 288 | 1188 |
| `scale_k_4.json`       | 5 | 4 | 5 | 10 | 15 | SAT    | 0.051 | 384 | 1568 |
| `scale_n_3.json`       | 3 | 2 | 5 | 10 | 15 | SAT    | 0.013 | 128 |  304 |
| `scale_n_4.json`       | 4 | 2 | 5 | 10 | 15 | SAT    | 0.017 | 160 |  526 |
| `scale_n_5.json`       | 5 | 2 | 5 | 10 | 15 | SAT    | 0.024 | 192 |  808 |
| `scale_n_6.json`       | 6 | 2 | 5 | 10 | 15 | SAT    | 0.032 | 224 | 1150 |
| `scale_n_7.json`       | 7 | 2 | 5 | 10 | 15 | SAT    | 0.036 | 256 | 1552 |
| `scale_n_8.json`       | 8 | 2 | 5 | 10 | 15 | UNSAT  | 0.043 | 288 | 2014 |
| `swap_3n_2d.json`      | 3 | 2 | 3 |  6 |  6 | SAT    | 0.007 |  56 |  128 |
| `tiny_2n_1d.json`      | 2 | 1 | 10|  5 |  6 | SAT    | 0.003 |  21 |   36 |

All SAT cases validate `PASS`. The two UNSAT outcomes are both *expected*:

- `scale_T_tight` ($T = 2$): two drones cannot revisit four non-depot cycle
  nodes within every 2-timestep window.
- `scale_n_8` ($n = 8,\, k = 2,\, T = 5$): the cycle is too long for two
  drones to keep every node fresh within the 5-step deadline.

No instance times out at the chosen sizes; the bottleneck dimension for
future scaling is $n$ (worst-case $\Phi_3$ is $kHn(n-1)$), in line with the
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
  (or the `--horizon` CLI flag). Wrapping a binary search over $H$ around
  `solve_instance` is a natural next step but is intentionally outside the
  current scope.

---

## 8. Summary

`mapf_surveillance` is a literal implementation of the reduction
$\mathrm{MDPS} \to \mathsf{QF\_LIA} + \mathsf{Bool}$ defined in §3. Each of
the seven constraint groups lives in its own typed, documented function in
`encoding.py`; the solver glues them together with Z3 via pySMT; the
validator independently re-checks every SAT result; and the benchmark
exercises the encoding across five scaling dimensions. The reduction is
polynomial-size and is proved sound and complete in §3.3, so a
satisfiability outcome from Z3 translates directly into a correct decision
for the surveillance problem.
