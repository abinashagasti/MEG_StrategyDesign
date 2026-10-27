# Audit: Section V-B simulations

Scope: everything was read-only. All new scripts, runs and plots are in `paper_audit/sim/`. Raw output: `sim_results.txt` (from `sim_audit.m`) and `feint_trend_log.txt`.

Confidence key: **H** = read directly or recomputed exactly; **M** = inferred; **—** = cannot be determined.

## Key findings

1. **The stored data reproduce every number in the paper** (times exact; distances within 0.0005). They also replay **bit-for-bit** with the current code.
2. **Example 4 actually ran with ρ = 0.12, not 0.21 (or 2.1).** So the condition the paper invokes, ρ > 4Δt = 0.20, is **not met** by the stored run.
3. **All three stored runs use Δt = 0.05.** Paper line 1448 says "all simulations use Δt = 0.025", which contradicts both the data and each example's own text.
4. **Example 4's claim "all three captures occur farther from the target" is false for E3 in the stored data.** E3 is at 1.3183 vs 1.3195, i.e. 1.2 mm closer. At Δt = 0.025 the claim holds.
5. **The Example 5 switch E4 → E3 is confirmed,** at t = 1.15 s.

## Item table

| # | Answer | Source | Conf. |
|---|---|---|---|
| **1** | **Ex 3** = `Simulation 1/sim1_file.mat`. **Ex 4** = `Simulation 4/sim1_evader_deviation.mat`. **Ex 5** = `Simulation 2/evader_sequence_switch_file.mat`. Each is byte-identical (md5) to its copy in `results/`, and the initial positions match the paper to 3 dp. Related files: `sim1.mat` (10 Aug, ρ = 0.2, same initial conditions as Ex 3/4) and `evader_sequence_switch.mat` (10 Aug, ρ = 0.2, same initial conditions as Ex 5) are earlier runs from which the initial conditions were taken. `Simulation 3/sim2_evader_deviation.mat` (1v4, T = (−0.600, −1.214), straight-to-target, ρ = 0.21) is **not one of the three V-B examples**. | md5; `history.*_positions(:,1)` | H |
| **2** | `main.m` → `Environment(n, timestep, tolerance, …)` → `env.simulate(10000, true)`, saved by the `store_history` block. The three were created with `init_source = "mat"` from `sim1.mat` / `evader_sequence_switch.mat` (`main.m:10–11, 57–67, 93, 108, 117–130`). Stored settings: `policy = "closest_next_step"`, `max_speed = 1`, `tolerance` and `evader_policy` per example (below). The exact `main.m` of 13 Aug was never committed (it sits between `da2fa05` of 24 Jul and `68ef4f1` of 19 Aug). However, rerunning the current `Environment.simulate` from the stored initial conditions and settings reproduces all three runs with **zero position difference and identical capture flags**. | `history.*`; git log; `[repro]` lines | H (settings, dynamics) / M (exact script state) |
| **3** | Δt = **0.05** in all three. The time vector is uniform, 0:0.05:…. | `history.time` | H |
| **4** | Ex 3 **0.12**, **Ex 4 0.12**, Ex 5 **0.12**. Ex 4 was neither 2.1 nor 0.21. Why: `main.m:104` tests `evader_policy == "straigth_to_target"` (misspelt, so never true), and its `tolerance = 0.21` assignment comes *after* the Environment is built (`main.m:93`), so it could not take effect anyway. Only the unrelated Simulation 3 file has 0.21. | `history.tolerance`; `main.m:93, 104–106` | H |
| **5** | Capture: `‖P−E_i‖ < ρ` (strict), checked first (`Environment.m:217, 227`). Invasion (`Environment.m:241–250`): `‖E_i−T‖ < r_inv` **and** `‖P−T‖ > 2·r_inv`. Current code: `r_inv = 1.1·Δt = 0.055` (`Environment.m:74`), committed 19 Aug. Last committed code before the runs: `r_inv = ρ = 0.12` (`git show da2fa05:Environment.m`). **Which rule was live on 13 Aug cannot be determined from git.** It does not matter for these runs: I checked both rules at every step and **neither would ever fire**, so the trajectories are identical either way. Neither rule is exactly the paper's "within Δt of T", because of the 1.1 factor and the extra pursuer-distance condition. | Environment.m; `[5]` lines | H / — (13 Aug rule) |
| **6** | The code has **one** pursuer controller: `closest_next_step` = nominal **plus fallback**. If the target is in P's dominance region (barrier ≥ 0, α = 1), P takes θ*_c; otherwise it heads to T (`Pursuer.m:206–214, 238–243`; `optimal_pursuer_heading` 63–89). So Examples 3 and 5 also ran the "robust" code. Because the fallback never triggered (item 9), their trajectories are identical to the nominal controller's. **Critical evader** = argmin over active i of `‖E_i + Δt·[cos ψ*_i; sin ψ*_i] − T‖` (`Pursuer.m:71–80`). **Lookahead = `timestep` = 0.05** at unit speed (`Environment.m:266`). | code | H |
| **7** | Ex 3 **optimal**, Ex 4 **straight_to_target**, Ex 5 **optimal**. Optimal = each evader runs its own 1v1 dominance check and, if P dominates (α = 1), heads to the intercept point on the PE bisector nearest T; otherwise straight to T (`Evader.m:71–80, 93–106`; `Environment.m:204–212`). | `history.evader_policy`; Evader.m | H |
| **8** | See the comparison rows below. **All match the paper.** The paper's convention is the time of the first sample with ‖P−E‖ ≤ ρ (`Simulation */data_extraction.m:83`). The code's `captured_evaders` flag is set one step (0.05 s) later, because the check runs at the start of the next step. | `sim_results.txt` `[8]`, `[cmp]` | H |
| **9** | Steps with the target outside P's dominance region (i.e. fallback steps): **Ex 3: 0, Ex 4: 0, Ex 5: 0.** | `[9]` | H |
| **10** | Minimum ‖E−T‖ over active evaders at decision steps: **Ex 3 1.122** (E2, t = 1.95). **Ex 4 1.201** (E2, t = 1.55). **Ex 5 0.157** (E4, t = 2.30). In Ex 5, E4 is then captured at 0.1075 from T; that sample counts as the capture, not as an active step. | `[10]` | H |
| **11** | Minimum pairwise separation of θ*_i between active evaders: **Ex 3 32.43°** (E1–E3, t = 0). **Ex 4 13.36°** (E1–E3, t = 0.70). **Ex 5 1.58°** (E1–E3, t = 0.50). | `[11]` | H |
| **12** | **Ex 3:** E3 → E2 (after capture, 0.85) → E1 (after capture, 2.00). **Ex 4:** E3 → E2 (0.80) → E1 (1.60). **Ex 5:** E2 → E1 (after capture, 0.55) → **E4 (after capture, 1.00) → E3 by threat switch at t = 1.15** → E4 (after capture, 1.60). **The E4 → E3 switch is confirmed.** E4 was critical for 3 steps (t = 1.00, 1.05, 1.10). | `[12]` | H |
| **13** | **Stored data:** E1 1.903 > 1.265 ✓, E2 1.151 > 1.095 ✓, **E3 1.3183 < 1.3195 ✗**. So "all three farther" is **false** for E3, by 0.0012. The paper's rounded 1.318 vs 1.32 also shows E3 closer. | `[13]` | H |
| **14** | Reruns at Δt = 0.025, ρ = 0.12 with the current code (invasion radius 0.0275). **Ex 3:** E3 0.850 / 1.3195, E2 2.000 / 1.0948, E1 **3.675 / 1.2862**. Fallback 0. Min ‖E−T‖ 1.108. Min separation 32.43° (t = 0). Sequence E3 → E2 → E1. **Ex 4:** E3 **0.775 / 1.3433**, E2 **1.550 / 1.2012**, E1 **2.850 / 1.9534**. Fallback 0. Min ‖E−T‖ 1.226. Min separation 14.81° (E1–E3, t = 0.725). Same order. **"All three farther than Ex 3" is TRUE at Δt = 0.025.** **Ex 5:** E2 0.550 / 0.4207, E1 1.000 / 0.5381, E3 1.600 / 0.8476, E4 **2.325 / 0.1386**. Fallback 0. Min ‖E−T‖ 0.162. Min separation 1.47° (E1–E3, t = 0.50). Sequence E2 → E1 → E4 → **E3 (threat switch at t = 1.175)** → E4, so the switch persists. Plots: `figs/rerun_example{3,4,5}_dt0025.png`; data: `rerun_*.mat`. | `[cmp]` RERUN blocks | H |
| **15** | Example 4 initial conditions, robust controller. Feint: flee directly away from P when ‖P−E‖ < 0.8, otherwise straight to T. ρ = 0.12, 40 s, at **Δt = 0.025** (and 0.05 as a check). The feint policy is not in the codebase, so `feint_run()` reproduces `Environment.step` line-for-line and replaces only the evader velocity. **Result: no captures, no invasion, 0 fallback steps.** Min ‖E−T‖ = **0.746 m** (0.725 at Δt = 0.05). The run reaches a **stalemate**: from about 20 s on, P sits within 0.045 m of T (0.086 at Δt = 0.05), the evaders hover about 0.76 m from P, and min ‖E−T‖ stays flat at 0.7461. The critical-evader choice chatters: **518 switches** (261 at Δt = 0.05). So the target stays safe, but there is no capture in finite time against this feint. Plots: `figs/feint_example4_dt{0025,0050}.png`. | `[15]`; `feint_trend_log.txt` | H (as implemented) |

## Stored vs paper, per example

| Example | Evader | Stored time (s) | Paper time | Stored ‖E−T‖ | Paper ‖E−T‖ | Match |
|---|---|---|---|---|---|---|
| **Ex 3** (Δt 0.05, ρ 0.12, optimal) | E3 | 0.85 | 0.85 | 1.3195 | 1.32 | ✓ |
| | E2 | 2.00 | 2.0 | 1.0946 | 1.095 | ✓ |
| | E1 | 3.70 | 3.7 | 1.2652 | 1.265 | ✓ |
| **Ex 4** (Δt 0.05, **ρ 0.12**, straight) | E3 | 0.80 | 0.8 | 1.3183 | 1.318 | ✓ (but ρ is not 0.21) |
| | E2 | 1.60 | 1.6 | 1.1512 | 1.151 | ✓ |
| | E1 | 2.90 | 2.9 | 1.9034 | 1.903 | ✓ |
| **Ex 5** (Δt 0.05, ρ 0.12, optimal) | E2 | 0.55 | 0.55 | 0.4207 | 0.421 | ✓ |
| | E1 | 1.00 | 1.0 | 0.5377 | 0.538 | ✓ |
| | E3 | 1.60 | 1.6 | 0.8452 | 0.845 | ✓ |
| | E4 | 2.35 | 2.35 | 0.1075 | 0.108 | ✓ |

## What to fix in the paper

- **Example 4:** state ρ = 0.12. At Δt = 0.05 this violates ρ > 4Δt (0.12 < 0.20). Two options: rerun at Δt = 0.025, where 4Δt = 0.10 < 0.12 and the "all farther" claim becomes true (item 14); or rerun at Δt = 0.05 with ρ = 0.21. The latter is not done here; say if you want it.
- **Line 1448** ("all simulations use Δt = 0.025") contradicts the stored runs and each example's text. Either change it to 0.05, or switch to the item-14 numbers.
- **Example 4's "all three captures farther" claim** is false for E3 in the stored run.
- **Description of the controller:** the code has a single controller. "Nominal" and "robust" are the same code, which behaves as nominal whenever the fallback is not triggered (never, in these runs).
