# MEG — Multi-Evader Target-Guarding on EV3 + OptiTrack

A pursuer must capture `n` independent evaders before any of them reaches a
target ("target guarding"). The strategy is developed in simulation and then run
on a real testbed: **OptiTrack (Motive) motion capture + ROS 2 (vrpn_mocap) +
LEGO EV3 kiwi-drive (3-wheel omni) robots**, all driven from MATLAB.

The core idea of the code: the **strategy is written once** (`Pursuer`, `Evader`,
`Environment`) and shared between simulation and hardware. The hardware layer
only swaps **sensing** (OptiTrack instead of a state integrator) and
**actuation** (EV3 motors instead of `pos += dt*v`). Nothing about robots,
Jacobians or yaw offsets leaks into the strategy.

---

## Prerequisites for hardware runs

1. **Motive** is streaming, and every rigid body you use (`pursuer`, `evader1`,
   `evader2`) exists and is currently tracked. The rigid-body **name is the ROS
   topic**: `/vrpn_mocap/<name>/pose`.
2. **VRPN bridge** is running:
   ```bash
   ros2 launch vrpn_mocap client.launch.yaml server:=192.168.0.118
   ros2 topic list           # expect /vrpn_mocap/<name>/pose for each bot
   ros2 topic hz /vrpn_mocap/pursuer/pose   # confirm a steady rate
   ```
3. Every EV3 is powered on and reachable (`ping <ip>`); IPs/serials live in
   `botConfigs.m`.
4. Each bot has been **calibrated** (`calibrate_yaw_offset`) — an uncalibrated
   bot has a `NaN` yaw offset and the code refuses to drive it.
5. **Run from the MATLAB Desktop**, not the VS Code MATLAB extension — long
   blocking hardware loops drop the extension's engine connection.

> **Coordinate frame:** floor plane is `(x, y)`, yaw about `z` (`up_axis = 'z'`).
> Positions/goals are in **mocap-frame metres**. Re-wanding, resetting the ground
> plane, or recreating a rigid body **invalidates every `yaw_offset`** — recalibrate.

---

## File reference

### Strategy + simulation (shared with hardware)
| File | What it is |
|---|---|
| `Environment.m` | Game logic: barrier / dominance test, termination, `step()` (Euler integration), `simulate()`. |
| `Pursuer.m` | Pursuer strategy. `heading_velocity(...)` with policies `closest_next_step` (default), `standard`, `squaresum`, `squaresump`, `heuristic`, `closest`. |
| `Evader.m` | Evader strategy (optimal escape / run-to-target). |
| `main.m` | Pure-simulation driver. Set positions + policy, calls `env.simulate()`. No hardware. |

### Hardware layer
| File | What it is |
|---|---|
| `BotHardware.m` | Everything physical for **one** bot: EV3 connection, mocap subscription, per-bot constants. Key methods: `sense`, `drive` (world-frame velocity), `drive_body`, `halt`. Owns dropout + frozen-feed handling. |
| `env_hardware.m` | `Environment` subclass. Overrides `step()` to sense → run the **unchanged** strategy → transform → actuate. Entry point `run()`. |
| `main_hardware.m` | Hardware twin of `main.m`: builds the bot configs, target, and calls `env.run()`. |

### Configuration + connection helpers
| File | What it is |
|---|---|
| `botConfigs.m` | **Single source of truth** for per-bot constants (IP, serial, wheel radius `r`, offset `L`, `yaw_offset`, `motor_max_radps`). `botConfigs("evader1")` returns one bot's config. |
| `getEv3.m` | Shared per-brick `legoev3` connection registry. Opens a brick **once** and reuses it (an EV3 allows only one connection). `getEv3(serial,ip,true)` forces reconnect. |
| `getMocapNode.m` | Returns a fresh `ros2node` each call (a node must **not** be reused — a stale one crashes MATLAB). |
| `readPose.m` | Converts a `PoseStamped` message → `[pos, yaw, raw]` for the chosen `up_axis`. |
| `stopMotors.m` | Standalone 3-motor stop helper (used by older scripts). |

### Calibration + testing
| File | What it is |
|---|---|
| `calibrate_yaw_offset.m` | Measures a bot's `yaw_offset` **and** its true speed (→ `motor_max_radps`). Function: `calibrate_yaw_offset("evader1")`. |
| `closedloop_optitrack_ev3motion.m` | Single-bot go-to-point via `BotHardware`. Function: `closedloop_optitrack_ev3motion("evader1",[x;y])`. |
| `lego_connect_test.m` | Bare EV3 connectivity + motor smoke test. |
| `ros2_test.m` | Bare ROS2 node + subscriber + `receive` smoke test. |

---

## Workflow 1 — Calibrate a bot

Run once per bot (and again after any Motive re-wand / rigid-body change).
Give the bot ~1 m of clear space ahead.

```matlab
calibrate_yaw_offset("evader1", push_time=3.0, push_speed=0.20)
```

It pushes body-forward a few times, measures which way the world says it went,
and prints two numbers. **Paste both into `botConfigs.m`** under that bot:

```matlab
'yaw_offset',      -1.5990, ...    % from the printout
'motor_max_radps', 5.5);           % from the printout — makes real m/s match the command
```

- `yaw_offset` aligns the bot's mechanical "forward" with the mocap frame.
- `motor_max_radps` calibrates true speed, so **all bots move at the same m/s**
  (the game assumes equal speeds, α = 1).
- Check the printed **spread** (want < ~10°) and **true speed %**.

## Workflow 2 — Basic single-bot test

Validates the whole stack (sense → control → world→body → Jacobian → motors) for
one bot before adding the game on top.

```matlab
closedloop_optitrack_ev3motion("evader1", [0; 0])
% options: vmax, tol, Kp, timeout, reset=true
```

Watch the `pos | yaw | err` trace: `err` should fall monotonically to `< tol`
and print `goal reached`. Pick goals inside the tracked volume (the start
position prints on connect).

## Workflow 3 — Run the target-guarding experiment

1. Confirm all three bots are calibrated in `botConfigs.m` (no `NaN` offsets).
2. Edit `main_hardware.m`: set the **target** (mocap frame), the arena geo-fence,
   `max_speed`, `tolerance`, and the pursuer policy.
3. Agent **start positions are read from mocap** — just place the bots on the
   floor; don't type positions.
4. Run:
   ```matlab
   main_hardware        % builds env from live poses, then env.run()
   ```
   `run()` plays the game until capture / an evader reaches the target / timeout,
   stops all motors, and plots the trajectories.

> `run()` is a **method**, so `onCleanup` stops every motor on a normal finish,
> an error, **or Ctrl-C**. Never drive the bots from a script-level loop.

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `Failed to connect to EV3` | Brick busy (one connection only). `clear all`, or reboot the brick; connections are shared via `getEv3` within a session. Confirm IP with `ping`. |
| MATLAB process crashes on `ros2subscriber` | Stale DDS state. Use the current `getMocapNode` (fresh node each call). If it persists, close MATLAB, `ros2 daemon stop`, clear `/dev/shm/fastrtps_*`, relaunch vrpn. |
| Bot spirals / drives off at an angle | Wrong/stale `yaw_offset` — recalibrate. |
| `pos`/`yaw` frozen, loop hangs | Rigid body lost; feed republishes last pose. `BotHardware.sense` detects this via the frame timestamp and treats it as a dropout. Fix tracking (coverage / markers). |
| Bot visibly spins while translating | Mechanical/motor asymmetry. Harmless for position control (yaw is re-measured each step); a heading-hold could cancel it. |
| Goal never reached, stalls near target | Motor deadband at low speed. Raise `tol`, or lower `motor_max_radps` to command higher duty. |
