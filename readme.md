# MEG — Multi-Evader Target-Guarding on EV3 + OptiTrack

A single **pursuer** must capture `n` independent **evaders** before any of them
reaches a **target** ("target guarding"). The pursuer strategy is developed in
simulation and run on a real testbed: **OptiTrack (Motive) motion capture +
ROS 2 (vrpn_mocap) + LEGO EV3 kiwi-drive (3-wheel omni) robots**, all driven
from MATLAB.

**Core design principle:** the strategy is written once (`Pursuer`, `Evader`,
`Environment`) and shared between simulation and hardware. The hardware layer
(`env_hardware`, `BotHardware`) only swaps **sensing** (OptiTrack instead of a
state integrator) and **actuation** (EV3 motors instead of `pos += dt*v`).
Nothing about robots, Jacobians, or yaw offsets leaks into the strategy.

Working folder: `/home/esb201/MEG_StrategyDesign`
(An older ROS1 implementation of a *different* algorithm lives in `/home/esb201/MEG` — reference only.)

---

## 1. Conventions (read once)

- **Frame:** floor plane is `(x, y)`, yaw about `z` (`up_axis = 'z'`). Positions
  and goals are in **mocap-frame metres**.
- **Quaternion -> yaw:** `q = [w x y z]`, `quat2eul(q,'ZYX')`, `yaw = eul(1)` (see `readPose.m`).
- **Kinematics (kiwi):** `J = (1/r)*[-L 1 0; -L -0.5 -sqrt(3)/2; -L -0.5 sqrt(3)/2]`,
  twist `[omega; vx; vy]` (vx forward, vy left). `omega = 0` — heading is not regulated.
- **World -> body:** rotate the world velocity by `-(yaw + yaw_offset)` before the Jacobian.
- **Operating speed:** `max_speed = 0.18 m/s` (shared by all agents, keeps the speed ratio alpha = 1).
- **Motive names = ROS topics:** a rigid body `evader1` -> `/vrpn_mocap/evader1/pose`.

> Re-wanding, resetting the ground plane, or recreating a rigid body **invalidates
> every `yaw_offset`** — recalibrate those bots.

---

## 2. One-time machine setup (persists across sessions)

These two fixes work around MATLAB EV3-support-package bugs. Do them **once**;
they survive restarts.

**(a) Widen the EV3 connection window** — without this, only the *first* WiFi
brick connects reliably:
```matlab
setpref('MathWorks_LEGO_EV3', 'IO_WAIT_PAUSE', 0.005);   % ~0.5 s unlock window
```
The stock default (`0.00001`) gives a ~1 ms window, so the 2nd/3rd brick's
handshake reply arrives too late and fails. Only affects read polling, so it
does not slow the motor-only control loop.

**(b) Patch the multi-brick tracker bug** — MATLAB's `trackWiFi` crashes when a
*second* WiFi EV3 connects (`tracker(end+1) = ip` on a cell). Patched file +
backup:
```
~/Documents/MATLAB/SupportPackages/R2025b/toolbox/realtime/targets/ev3io/+realtime/+internal/trackWiFi.m
```
Fix: line ~19 `tracker(end+1) = ip;` -> `tracker{end+1} = ip;`, and line ~25
`tracker(loc) = '';` -> `tracker(loc) = [];`. Backup is `trackWiFi.m.orig`.
**A support-package update may overwrite this — re-apply from the backup.**

---

## 3. Per-session prerequisites

1. **Motive** streaming; rigid bodies `pursuer`, `evader1`, `evader2` created and
   **tracked** (asymmetric marker constellations so orientation doesn't flip).
2. **VRPN bridge** up:
   ```bash
   ros2 launch vrpn_mocap client.launch.yaml server:=192.168.0.118
   ros2 topic list          # /vrpn_mocap/{pursuer,evader1,evader2}/pose present
   ros2 topic hz /vrpn_mocap/pursuer/pose   # steady rate
   ```
3. EV3 bricks powered on, **fresh/matched batteries**, reachable (`ping <ip>`).
4. **Run from the MATLAB Desktop**, not the VS Code extension (long blocking
   hardware loops drop the extension's engine).

---

## 4. File reference

### Strategy + simulation (shared)
| File | Role |
|---|---|
| `Environment.m` | Game logic: barrier/dominance test, termination, `step()` (Euler), `simulate()`. |
| `Pursuer.m` | Pursuer strategy. Default policy `closest_next_step`; also standard/squaresum/squaresump/heuristic/closest. |
| `Evader.m` | Evader strategy. |
| `main.m` | Simulation driver. `init_source` = "manual" / "mat" / "mocap" (see below). |

### Hardware layer
| File | Role |
|---|---|
| `BotHardware.m` | One bot: EV3 connection, mocap sub, calibration constants, `sense`/`drive`/`halt`. Owns dropout + frozen-feed detection + stiction deadband. |
| `env_hardware.m` | `Environment` subclass. `step()` = sense -> unchanged strategy -> transform -> actuate. Entry point `run()`. |
| `main_hardware.m` | Runs the real game; reads live start poses; saves history. |

### Config + connection helpers
| File | Role |
|---|---|
| `botConfigs.m` | **Single source of truth** for per-bot constants (IP, serial, wheel geometry, `yaw_offset`, `motor_max_radps`, optional `motor_deadband`). |
| `getEv3.m` | Shared per-brick `legoev3` connection (one connection per brick, reused). |
| `getMocapNode.m` | Returns a fresh `ros2node` each call (never reuse a stale node -> crash). |
| `readPose.m` | `PoseStamped` msg -> `[pos, yaw, raw]`. |
| `readMocapPoses.m` | One-shot read of several rigid bodies' positions (create-all-then-read). |

### Calibration + manual tools
| File | Role |
|---|---|
| `calibrate_yaw_offset.m` | Measures a bot's `yaw_offset` **and** true speed (-> `motor_max_radps`). |
| `closedloop_optitrack_ev3motion.m` | Single-bot go-to-point via `BotHardware` (the stack sanity test). |
| `find_deadband.m` | Ramps duty to find a bot's from-rest stiction threshold (-> `motor_deadband`). |
| `motor_step_response.m` | Measures actuator rise time / settling from mocap. |
| `connect_ev3s.m` | Connect the bricks only (no ROS/game) — connection debug + pre-cache. |
| `lego_connect_test.m`, `ros2_test.m` | Bare EV3-only / ROS-only smoke tests. |

Run outputs are saved to `results/*.mat` (gitignored).

---

## 5. Redo the experiments — full procedure

### Step 0 — bring-up
- Do the **one-time setup** (Section 2) if this machine hasn't had it.
- Start Motive + the VRPN bridge; confirm all three pose topics (Section 3).
- Power the bricks; check batteries are healthy and **similar** across bots.

### Step 1 — connect the bricks
```matlab
clear all
connect_ev3s          % connects all three, retries, prints battery levels
```
- All three should report `OK`. If one keeps failing: `ping` it; reboot that
  brick; confirm its serial in `botConfigs.m`. See Troubleshooting.
- This **caches** the connections, so the later `main_hardware` reuses them.

### Step 2 — calibrate each bot (at the operating speed)
For **each** bot, give it ~1 m clear space and run:
```matlab
calibrate_yaw_offset("pursuer", push_speed=0.18)
calibrate_yaw_offset("evader1", push_speed=0.18)
calibrate_yaw_offset("evader2", push_speed=0.18)
```
Paste the two printed values into `botConfigs.m` under that bot:
```matlab
'yaw_offset',      <value>, ...
'motor_max_radps', <value>);
```
- Calibrate at `push_speed` = the game `max_speed` (0.18) so command = actual there.
- Want tighter alpha = 1? Keep batteries matched and recalibrate at session start.
- `spread` should be a few degrees; a large spread means the rigid body's yaw is
  flipping (fix marker asymmetry) — not a calibration problem.

### Step 3 — (optional) stiction deadband for a slick-wheel bot
If a bot **stalls from rest / needs a push** on the tiles:
```matlab
find_deadband("evader1")     % note the duty % where it first moves from rest
```
Add to that bot in `botConfigs.m`: `'motor_deadband', <that duty + ~3>`.
(Leave it off for bots that don't stall. Note: at `vmax = 0.18` most bots have
enough duty that the deadband rarely triggers.)

### Step 4 — verify each bot single-handed
```matlab
closedloop_optitrack_ev3motion("pursuer", [0;0])
closedloop_optitrack_ev3motion("evader1", [0;0])
closedloop_optitrack_ev3motion("evader2", [0;0])
```
Each should drive to the goal with `err` falling to `< tol` and print
`goal reached`. Pick goals inside the tracked volume (start pos prints on connect).

### Step 5 — run the game
Edit `main_hardware.m` game setup:
```matlab
target_position = [0; 0.5];        % mocap frame
arena_limits    = [-1.5 1.5 -1.5 1.5];
'tolerance',  0.5, ...             % capture radius
'max_speed',  0.18, ...            % shared speed
'policy',     "closest_next_step", ...
```
Place the three bots on the floor (**start positions are read from mocap** — do
not type them); position so the pursuer can defend the target. Then:
```matlab
main_hardware
```
- Read the printed **initial-state block**: `barrier ... -> pursuer wins` means
  proceed; `-> EVADERS win` means reposition and rerun.
- It plays until capture / an evader reaches the target / timeout, stops all
  motors, plots the trajectories, and **saves `results/run_<timestamp>.mat`**.
- **Hand on Ctrl-C** — `onCleanup` stops every motor on any exit.

### Step 6 — analyse / replay in simulation
`main.m` can start from three sources via `init_source`:
```matlab
init_source = "manual";   % hand-typed positions
init_source = "mat";      % first frame of results/<file>.mat (set mat_file)
init_source = "mocap";    % live bot positions right now
```
Use `"mat"` to replay a hardware run's start in sim and compare, or `"mocap"` to
seed a sim from the current physical layout.

To pull data from a saved run for plotting (e.g. TikZ/pgfplots), load it:
```matlab
S = load('results/run_XXXX.mat'); h = S.history;
% h.pursuer_positions (2xT), h.evader_positions (2 x n x T), h.target_position, ...
```

---

## 6. Calibration & tuning quick reference

| Quantity | What it does | How to set |
|---|---|---|
| `yaw_offset` | Aligns body "forward" with the mocap frame | `calibrate_yaw_offset` -> paste |
| `motor_max_radps` | command speed = actual speed (sets alpha=1) | `calibrate_yaw_offset` at op-speed -> paste |
| `motor_deadband` | Min duty so a nonzero command breaks stiction | `find_deadband` -> paste (only if it stalls) |
| `max_speed` / `vmax` | Operating speed | edit in `main_hardware` / pass to `closedloop` |
| `Kp`, `tol`, `dt` | go-to-point gain / stop radius / period | `closedloop` options |

**Battery note:** motor speed drifts with battery voltage (open-loop). Keep bots
at similar charge so the *ratio* alpha stays ~1; recalibrate at session start for
tighter matching. Exact per-run speed calibration is not worth it — see the
matched-battery approach.

---

## 7. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Only the 1st WiFi brick connects; 2nd/3rd "Failed to connect" | The 1 ms unlock window. Set `IO_WAIT_PAUSE` (Section 2a). |
| `Conversion to cell from char` in `trackWiFi` | Unpatched multi-brick bug (Section 2b). Apply the patch. |
| `Failed to connect to EV3` (single brick) | Stuck connection slot -> reboot that brick; `clear all`; `ping` it; confirm serial. Don't run `lego_connect_test` right before the game (it holds the slot). |
| MATLAB process dies on `ros2subscriber` | Stale ROS node reuse. `getMocapNode` now makes a fresh node each call. If persists: close MATLAB, `ros2 daemon stop`, clear `/dev/shm/fastrtps_*`, relaunch vrpn. |
| "Transport stopped" reading several bodies | Subscriber churn. Use `readMocapPoses` (creates all subs first, then reads). |
| Bot spirals / drives at an angle | Wrong/stale `yaw_offset` — recalibrate. Steady curve near goal = small offset bias. |
| `pos`/`yaw` frozen, loop hangs | Rigid body lost; last pose republished. `BotHardware.sense` catches it via the frame timestamp. Fix tracking (coverage / marker asymmetry). |
| Rigid body untracked in-range, fixed by recreating it | Marker constellation too symmetric/sparse -> Motive can't re-acquire. Use 4+ asymmetric, non-coplanar markers. |
| Bot swerves / rotates while translating | Mechanical spin + control staleness. Raise `max_speed` (0.18 works) — more ground per unit rotation straightens the path. |
| Bot stalls from rest / needs a push | Low-grip wheels on tiles. Set `motor_deadband` (Section, Step 3) and/or raise `vmax`. |
| Speeds don't match between bots | Different battery levels — match charge; recalibrate `motor_max_radps` at the operating speed. |

---

## 8. Notes on the physical setup

- **Cameras:** 8-camera ring; re-wand after any camera is moved (it invalidates
  calibration *and* every `yaw_offset`).
- **Markers:** 11 total across 3 bots (4 + 4 + 3). Each rigid body must be a
  **distinctly asymmetric, non-coplanar** shape so Motive doesn't swap/flip them.
- **Floor:** tiles (low grip). Older wheels grip better than newer ones — hence
  the deadband / higher-speed workarounds for the slick bot.
