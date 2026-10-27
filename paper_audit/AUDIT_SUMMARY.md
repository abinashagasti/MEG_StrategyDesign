# Audit: experiment code and logged data vs. paper claims

Scope: everything was read-only. No existing code or data file was modified. The analysis lives in `paper_audit/` only.

**Where the data actually is.** No hardware run is stored in this repo. `results/*.mat` and `scratch_flip_results.mat` are simulations or empty. The two hardware logs used by the paper are in the paper folder (read in place, not copied):

- `…/Paper Submissions/Multiplayer Evader/Modified Experiment Images/Experiment 1/Exp1_3.mat`
- `…/Experiment 2/Exp2_1.mat`

**Which code produced them.** The runs were made on 23 Jul 2026. Both videos (`Experiment Videos/exp1_video.mov`, `exp2_video.mov`) are stamped 22:00 and 22:04 IST. Commit `888bcdf`, "completed 2 basic runs", followed at 22:18 IST. `env_hardware.m` and `main_hardware.m` are unchanged since that commit. `Pursuer`/`Evader`/`Environment` changed later only by additions that the hardware path does not use. The only `BotHardware` change removed an unused `hold_heading` option, which was `false` for every bot. Calibration constants are unchanged. Line numbers below refer to HEAD, which matches the experiment code on every line cited.

**How time is logged.** This follows from the code in `env_hardware.m:128–142,174–242` and matters for items 7, 14, 16, 17 and 20–21.

- `history.time(j)` is `toc` taken **after** iteration j−1 has finished, so it includes `pause(0.05)`.
- `positions(:,j)` were sensed at the **start** of iteration j−1, i.e. right after `time(j-1)`.
- So each logged position is about one iteration *older* than its stamp.
- Sample 1 is the constructor read (before `tic`). Sample 2 is sensed at about t = 0, before any motor command. Samples 1 and 2 are identical in both logs, which confirms this.

Throughout, "t_sense(j)" = `time(j-1)`.

Confidence key: **H** = read directly from code or data; **M** = inferred from data with stated caveats; **—** = cannot be determined.

| # | Answer | Source | Conf. |
|---|---|---|---|
| **1** | **Hardware path:** `main_hardware.m` (sets target and options, runs, saves `results/run_<stamp>.mat`), `env_hardware.m` (Environment subclass: sense → capture → invasion → decide → drive → pause), `BotHardware.m` (one robot: OptiTrack subscriber, freshness check, world→body→kiwi Jacobian→motor %), `botConfigs.m` (per-robot IP, yaw_offset, motor_max_radps). **Strategy (shared with sim):** `Environment.m` (barrier/dominance, capture/termination, Euler sim), `Pursuer.m` (policies; `closest_next_step` used), `Evader.m` (1v1 optimal / straight-to-target). **Sim/analysis:** `main.m`; `find_flip_ic.m`, `check_one_step_flip.m`, `find_feasible_ics.m`, `flip_feasibility_constraints.m` (Aug 2026 search for PDR→EDR flip ICs, sim only). **Calibration/tests:** `calibrate_yaw_offset.m` (yaw offset + true speed), `closedloop_optitrack_ev3motion.m` (single-bot go-to-point), `motor_step_response.m` (rise time; results not saved), `find_deadband.m`, `connect_ev3s.m`, `lego_connect_test.m`, `ros2_test.m`. **Helpers:** `getEv3.m`, `getMocapNode.m`, `readPose.m`, `readMocapPoses.m`. **There are no EV3-side programs.** MATLAB writes motor duty over WiFi via the `legoev3` support package. Paper-folder scripts: `Experiment {1,2}/data_extraction.m` (export CSVs; see item 17), `Exp1/mat_to_tikz_data.py`, `matread.py`, `Experiment Videos/vid_processing.py` (overlay frames from video). | repo files; paper folder | H |
| **2** | **Exp1_3.mat**: `history` with time (1×19), pursuer_positions (2×19), evader_positions (2×2×19), captured_evaders (2×19), target_position, step_count = 18, terminated, termination_reason "all_evaders_captured", max_speed 0.18, tolerance 0.5, policy "closest_next_step". 19 samples, 0–6.181 s. **Exp2_1.mat**: same fields, 24 samples, 0–9.775 s. **Derived:** `Experiment 1/*.csv` and `Experiment 2/*.csv` (trajectories, dist_PE, dist_TE). `Exp1/*.dat` are TikZ exports of Exp1_3 (same start). **Videos:** `exp1_video.mov` (7.13 s), `exp2_video.mov` (11.51 s). **Not hardware:** `results/sim1–5.mat`, `sim{1,2}_file.mat`, `sim{1,2}_evader_deviation.mat`, `evader_sequence_switch{,_file}.mat`. These are all unit-speed sims with Δt = 0.05 and 3–4 evaders. `scratch_flip_results.mat` is an empty struct. **No OptiTrack/Motive exports (.tak/.csv), no mocap timestamps, no console logs exist anywhere searched.** | `paper_audit/inventory_log.txt`, `results_exp*.txt` | H |
| **3** | **Exp 1 = Exp1_3.mat**: T (0, 0.5), P (−0.879, 0.345), E1 (−0.404, −0.979), E2 (0.973, −0.864). All match the paper. **Exp 2 = Exp2_1.mat**: T (−0.5, 1), E1 (−1.234, −0.601), E2 (1.060, −0.748) match. **P = (+0.0195, 0.2669) in the data vs (−0.02, 0.267) in the paper: sign error in the paper.** | `history.*_positions(:,1)` | H |
| **4** | Free-running loop with a fixed `pause(env.timestep)` at the end of each iteration. There is no timer and no rate control. Period = execution time (3 blocking `receive()` calls + 9 EV3 WiFi motor writes + compute) + 0.05 s. `tic/toc` is used only to stamp the log and enforce `max_time`. Intended period = 0.05 s. | `env_hardware.m:241`, `:128,133,141`; `main_hardware.m:39` | H |
| **5** | **Δt**: `timestep = 0.05` (not 0.2). It sets only the pause and the strategy lookahead, which is `max_speed·timestep = 0.009 m`. **ρ**: `tolerance = 0.5` ✓. Capture check: `‖P−Ei‖ < ρ`. **Invasion**: the game ends if `‖Ei−T‖ < ρ = 0.5` **and** `‖P−T‖ > 2ρ = 1.0` (not 0.2 m). The 0.2-type radius `target_capture_tolerance = 1.1·Δt` is sim-only and was added 19 Aug, after the experiments. | `main_hardware.m:39–41`; `env_hardware.m:207–208, 226`; `Environment.m:214–221`, `:74` | H |
| **6** | Per iteration: (1) sense P, E1, E2 from mocap (`sense_all`). On a bad frame: halt all, pause, retry. (2) arena check. (3) **capture check** (`updateTermination`), then halt newly captured evaders, then end if all are captured. (4) **invasion check**. (5) dominance/win flag. (6) pursuer heading, then all evader headings. (7) drive P, then active evaders. (8) `pause(0.05)`. **Capture is checked before invasion.** | `env_hardware.m:181–241` | H |
| **7** | Capture times are **wall-clock** `toc` (MATLAB), stamped at the **end of the iteration** in which capture was detected (`history.time(k+1)`). They are neither loop-count×Δt nor OptiTrack time. That is why 3.723 etc. are not multiples of 0.2. The paper's script reports `time(first idx with dist ≤ ρ)`. Because of the timing model above, the position that triggered capture was measured ≈ one iteration (0.30–0.90 s) **before** the reported time. | `env_hardware.m:141–143`; `Experiment */data_extraction.m` | H |
| **8** | Only `history.time` (toc after each iteration) is logged. **Not logged:** OptiTrack header stamps (used only for the freshness check, `BotHardware.m:149–154`), loop durations, compute time, commanded headings/velocities/motor %, yaw, dropout warnings (console only), battery levels. | `env_hardware.m:120–153` | H |
| **9** | `drive(max_speed·[cosθ; sinθ])`, a world-frame velocity in m/s. It is rotated into the body frame by −(yaw+yaw_offset) using the **measured yaw each iteration**, then mapped through the kiwi Jacobian `J = (1/r)[−L 1 0; −L −½ −√3/2; −L −½ √3/2]` with ω = 0 (r = 0.025, L = 0.14). Wheel rad/s becomes `pct = 100·ω_wheel/motor_max_radps`, clamped to ±90 and written to `motor.Speed` (duty %). | `BotHardware.m:164–241` | H |
| **10** | The same commanded speed is used for all robots: `max_speed = 0.18 m/s`. Per-robot `motor_max_radps` (P 8, E1 9.2, E2 11.4) was calibrated to make actual speed ≈ command. The strategy itself is unit-speed with α = 1 (`Environment.m:57–58,75`). The *achieved* speeds differ (item 14). | `main_hardware.m:41`; `botConfigs.m:42,51,60` | H |
| **11** | **Open loop on the robot.** It receives duty %; there is no speed or heading (ω) controller (`omega = 0`). The only feedback is the central loop: the heading is recomputed from mocap each iteration and the world→body rotation uses the measured yaw. The readme confirms speed "drifts with battery voltage (open-loop)". | `BotHardware.m:196–200`; `readme.md` §6 | H |
| **12** | `closest_next_step`: nominal **plus fallback**. If the target is in the pursuer dominance region (barrier ≥ 0, α = 1), the pursuer takes θ*_c of the **critical evader**, else it heads straight to T (`Pursuer.m:83–86`, `238–243`). Critical evader = the one whose position after a lookahead step along its optimal ψ* is closest to T: `evader_next = E_i + timestep·[cos ψ*; sin ψ*]; if ‖evader_next−T‖ < closest … closest_evader = i` (`Pursuer.m:71–80`); `theta = theta_values(closest_evader)` (`:81`). On hardware timestep = lookahead = 0.009 m (`env_hardware.m:226–230`). **Replay of the logs:** the fallback never triggered (win = 1 at every sample). The critical evader was E1 until its capture, then E2, in both runs. | `Pursuer.m:63–89, 206–244`; `results_exp*.txt` | H (code) / H (replay) |
| **13** | Each evader runs its own 1v1 check against P (`Environment.m:207–210`). If it is in P's dominance region with α = 1, it heads to the intercept point I on the perpendicular bisector of PE nearest T (`Evader.m:71–80`); otherwise it heads to T. Evader commands use α = 1 even though physical speeds differ. Policy = "optimal" (default, `Environment.m:78`). Confirmed: in both runs dominance always held, so evaders always used the intercept heading. | `Evader.m:46–109`; `Environment.m:204–212` | H |
| **14** | Speed from consecutive positions over sensing intervals, movmedian(3). Excludes t < 1 s, motion after an agent's capture, and the 1 s after each capture for P. **Exp 1:** P **0.252 ± 0.005**, E1 **0.219 ± 0.029**, E2 **0.223 ± 0.043** m/s; P/E1 = 1.15, P/E2 = 1.13. **Exp 2:** P **0.232 ± 0.020**, E1 **0.250 ± 0.016** (one glitch sample removed, see 17), E2 **0.206 ± 0.024**; P/E1 = 0.93, P/E2 = 1.13. **All robots moved at ≈ 0.21–0.25 m/s: faster than the 0.18 command, far below 1 m/s.** The speed ratio was not 1: E/P ranged 0.87–1.08. Using the log stamps instead of sensing times gives 0.21–0.26 m/s, the same conclusion. | `figs/exp*_speed.png`; `results_exp*.txt` | M (±1 frame timing jitter) |
| **15** | Iteration (= log) period. **Exp 1:** mean 0.343, std 0.125, min 0.230, max 0.695 s (median 0.285 excl. first). **Exp 2:** mean 0.425, std 0.197, min 0.262, max 0.924 s (median 0.330). The first interval (≈ 0.7 s) includes motor `start()`. The achieved period is ≈ 6× the coded 0.05 s and ≈ 1.5–2× the paper's 0.2 s, with jitter up to 0.9 s. **OptiTrack frame rate: cannot be determined** (no stamps logged; Motive settings not in repo). | `figs/exp*_dt.png` | H / — |
| **16** | Cannot be measured below the ~0.3 s sample resolution. Two observations bound it: (a) at the heading switch after the first capture, the pursuer moved along the **old** heading for the entire next interval (Exp 1: −29.9° vs old command −26.3°, new 29.7°; Exp 2: 167.8° vs old 176.9°, new 8.9°) and was only partly turned in the interval after that; (b) Exp 1 E1 kept moving 0.061 m (~0.17 m/s) in the interval after its halt was issued. **Effective command-to-motion delay ≈ one iteration (≈ 0.3–0.36 s).** Actuation (WiFi/motor) delay cannot be separated from staleness of the mocap message. In steady motion, lag-0 vs lag-1 heading errors are indistinguishable because headings change slowly. | `results_exp*.txt` item 20 block; logged positions | M |
| **17** | Recomputed with `‖P−Ei‖ < 0.5` on raw logs. **Exp 1:** E1 j = 13, **3.723 s**, ‖E1−T‖ = **0.967** ✓; E2 j = 19, **6.181 s**, **0.776** ✓. **Exp 2:** E1 j = 13, **4.828 s** ✓, but raw ‖E1−T‖ = **0.727 m vs paper 0.974**; E2 j = 24, **9.775 s**, **0.871** ✓. **Why Exp 2 E1 differs:** sample 13 of E1 is a one-frame mocap glitch. E1 jumps 0.33 m to (−1.019, 0.491) and back. `Experiment 2/data_extraction.m:12` overwrites it by hand (`evader1_positions(:,13) = evader1_positions(:,14)`), and the paper's 0.974 comes from that edit. **The live capture decision was triggered by the glitched frame** (‖P−E1‖ = 0.136). With the replaced position, ‖P−E1‖ ≈ 0.39 < 0.5, so the capture is still consistent, but the true sample-13 position is unknown. **Physical timing:** the triggering positions were measured at 3.365 / 5.584 s (Exp 1) and 4.471 / 8.874 s (Exp 2). Each capture happened between that and the previous measurement (brackets in `results_exp*.txt`). | `results_exp*.txt`; `inspect_exp2.m` output; `data_extraction.m:12` | H |
| **18** | Minimum ‖Ei−T‖ while active (always at the capture sample): **Exp 1** E1 **0.967**, E2 **0.776** m. **Exp 2** E1 **0.727** raw (0.974 with the glitch replaced; the previous clean sample is 1.054), E2 **0.871** m. Exp 1 E1 coasted to 0.924 m *after* capture. | `results_exp*.txt` | H |
| **19** | `‖P−T‖ ≤ ‖Ei−T‖` for every active evader at every logged sample. **It never fails.** Minimum margin 0.459 m (Exp 1, sample 18), 0.491 m (Exp 2, sample 12). Caveat: only checked at ~0.3 s samples. | `figs/exp*_distances_dominance.png` | H |
| **20** | **Exp 1:** the command turned 56°; the pursuer was within 10° of the command from the interval starting **0.69 s** after the capture measurement, having travelled **0.155 m** first. **Exp 2:** the command turned 168°; within 10° after **1.61 s** and **0.294 m**. Most of the Exp 2 delay is a residual 10–13° offset that converges slowly, not the turn itself; the turn happened within ≈ 0.36–0.95 s. | `results_exp*.txt` item 20 block | M (0.3 s resolution) |
| **21** | Angle between the recomputed command and the measured motion direction over the following interval (moves < 1 cm skipped). **Exp 1:** P mean 7.7° (median 3.5°, max 59.7° at the switch; ≈ 3° excluding the two post-switch intervals), E1 10.1° (max 21.0°), E2 11.0° (max 15.0°). **Exp 2:** P 14.6° (median 7.5°, max 158.9° at the switch), E1 10.6° (max 17.6°), E2 12.0° (max 19.0°). The evaders show a consistent ~10–11° bias, consistent with the noted yaw-offset/swerve issues (item 26). | `figs/exp*_heading_error.png` | M (commands recomputed, not logged) |
| **22** | Minimum angular separation of θ*_i between the two active evaders (replayed with `Pursuer.optimal_headings_Ei`): **Exp 1: 14.7°** (at start, growing to 48° by E1's capture). **Exp 2: 163.3°** (stays 163–179°). | `results_exp*.txt` | H (replay of logged states) |
| **23** | **Not logged on hardware.** Offline replay of the per-iteration strategy computation (pursuer + both evaders) on this Mac (MATLAB R2023b, `timeit`): Exp 1 mean 0.28 ms (max 0.77 ms, includes JIT warm-up), Exp 2 mean 0.023 ms. This is negligible against the 0.3 s iteration. The lab machine and its sensing/WiFi times cannot be determined. | `audit_experiments.m` `step_compute` | M (different machine) |
| **24** | `wheel_radius = 0.025 m`, `wheel_offset` (centre-to-wheel) `L = 0.14 m`, both marked **"nominal"** (not measured per bot). Legacy unused `wheel_centre_radius = 0.15` in Pursuer/Evader. Geofence `arena_limits = [−1.5 1.5 −1.5 1.5]` m (tracked area, not stated physical arena). Robot body radius: **not in code** (only "big bots" comment for ρ). Readme: 8-camera ring, 11 markers (4+4+3), tiled floor. | `botConfigs.m:39–40`; `main_hardware.m:36,40`; `Pursuer.m:24`; `readme.md` §8 | H / — (robot radius) |
| **25** | **No additional hardware runs are stored for this controller.** The file names `Exp1_3` / `Exp2_1` imply other attempts existed, but none were found in the repo, paper folder, Desktop, Documents or Downloads. Their existence and content cannot be determined. Not part of this controller: `Videos/` (20 clips, 23–26 Apr 2024: away/target capture with standard/heuristic policies and closed-loop demos) and `Plotting for Paper/*.csv`, both from the older `Multiplayer_Reach_Avoid` code (a different algorithm). The 12 `results/*.mat` are simulations (Sims 1–4 in the paper folder). No 1v3 hardware attempt is stored. | searches; `mdls` dates | H (what exists) / — (unstored attempts) |
| **26** | At the experiment commit `888bcdf`, `botConfigs.m`: pursuer `yaw_offset` "TODO: stale after the re-wand -- re-measure"; pursuer `motor_max_radps` "TODO: measured ~47% of commanded -> expect ~7.5" (left at 8); evader wheel geometry "TODO: measure"; evader2 "this bot SWERVES -> set true once tuned" (`hold_heading` left false). Readme: open-loop speed drifts with battery, speed mismatch between bots (match charge/recalibrate), robots rotate while translating, low-grip tiles cause stalls, frozen mocap frames, multi-brick WiFi connection bugs (patched), rigid-body yaw flips. No notes about WiFi drops during runs. | `git show 888bcdf:botConfigs.m`; `readme.md` §6–8 | H |

## Implications for the revision

1. **Δt = 0.2 s is not what ran.** The code period was 0.05 s. The achieved period was 0.29–0.33 s (median), with jitter from 0.23 to 0.92 s.
2. **Invasion radius 0.2 m is not what ran.** The hardware terminates on `‖E−T‖ < 0.5` with `‖P−T‖ > 1.0`.
3. **Speed:** measured 0.21–0.25 m/s, not 1 m/s. The speed ratio was not exactly 1 (0.87–1.08).
4. **Exp 2:** P₀ has a sign typo, and the 0.974 m capture distance depends on a manual replacement of a glitched mocap sample. Disclose it or report the raw 0.727 m.
5. **Reported capture times** are wall-clock times at the end of the detecting iteration. They are ≈ one iteration later than the measurement that triggered capture.

## Files

- `audit_experiments.m`: items 14–23 → `results_exp1.txt`, `results_exp2.txt`, `figs/*.png`, `run_log.txt`
- `inventory_mat.m`: item 2 → `inventory_log.txt`
- `dump_exp.m`, `inspect_exp2.m`: raw per-sample tables
- `figs/`: `exp{1,2}_trajectory_headings`, `_speed`, `_dt`, `_distances_dominance`, `_heading_error`

## Follow-ups (`followup.m` → `followup_log.txt`)

**F1. Max per-iteration displacement δ.** Measured over sensing intervals while each robot is driven. Interval j→j+1 spans [time(j−1), time(j)].

| Run | P | E1 | E2 | δ (max over robots) |
|---|---|---|---|---|
| Exp 1 | 0.105 m (dt 0.415 s) | 0.070 m (dt 0.300 s) | 0.081 m (dt 0.358 s) | **0.105 m** |
| Exp 2 | 0.236 m (dt 0.924 s) | 0.236 m (dt 0.924 s)* | 0.161 m (dt 0.924 s) | **0.236 m** |

\*The raw value is 0.328 m, which is the glitch frame. The table excludes intervals 12→13 and 13→14. The Exp 2 maxima all fall in one 0.924 s stalled iteration (interval 8→9, 2.447–3.371 s). After E1's capture in Exp 1, E1 coasted another 0.061 m.

**F2. ‖P−E‖ at each capture sample j and at j−1** (ρ = 0.5):

| Capture | j−1 | j | ρ − d at j |
|---|---|---|---|
| Exp 1 E1 | 0.583 | 0.485 | 0.015 |
| Exp 1 E2 | 0.547 | 0.456 | 0.044 |
| Exp 2 E1 | 0.506 | 0.136 raw (0.390 with the paper's edit) | 0.364 raw (0.110 edited) |
| Exp 2 E2 | 0.597 | 0.498 | 0.002 |

**F3. Exp 2, E1 around the glitch:**

| Sample | t_sense (s) | E1 | P | ‖P−E1‖ | ‖E1−T‖ |
|---|---|---|---|---|---|
| 12 (clean, before glitch) | 4.209 | (−1.266, 0.276) | (−0.842, 0.553) | 0.506 | 1.054 |
| 13 (glitch) | 4.471 | (−1.019, 0.491) | (−0.906, 0.568) | 0.136 | 0.727 |
| 14 (clean, after; E1 halted) | 4.828 | (−1.242, 0.369) | (−0.980, 0.584) | 0.339 | 0.974 |
| Paper edit (E1@14, P@13) | — | | | 0.390 | 0.974 |

From sample 12 to sample 14, E1 moved 0.097 m in 0.62 s.
