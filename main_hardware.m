% MAIN_HARDWARE  Plays the MEG pursuit-evasion game on the real EV3 bots.
%
% This is the hardware twin of main.m: same game, same strategy, same Pursuer
% and Evader classes. Only sensing (OptiTrack) and actuation (EV3) differ.
%
% Before running:
%   1. Motive is streaming and every rigid body below is tracked.
%   2. ros2 launch vrpn_mocap client.launch.yaml server:=192.168.0.118
%      (check with:  ros2 topic list  ->  /vrpn_mocap/<name>/pose )
%   3. Every bot is powered on and reachable at the IP in its config.
%   4. Each bot's yaw_offset has been measured with calibrate_yaw_offset.m
%      FOR THAT BOT (an uncalibrated bot spirals; the config below refuses NaN).
%
% NOTE: do NOT put `clear` at the top of this script. The EV3 and ROS2
% connections are held on `env` (and in getMocapNode's persistent node);
% clearing them forces a slow reconnect every run. Use `clearvars -except env`.

clc

%% ================= PER-BOT CONFIG =================
% Pulled from botConfigs.m -- the single source of truth, kept current by the
% calibration workflow (yaw_offset, motor_max_radps, motor_deadband per bot).
% Do NOT re-type constants here; edit botConfigs.m instead.
pursuer_cfg = botConfigs("pursuer");
evader_cfgs = {botConfigs("evader1"), botConfigs("evader2")};
% evader_cfgs = {botConfigs("evader1")};

%% ================= GAME SETUP =================
% Agent start positions are NOT set here: they are read from mocap when the
% environment is built. Only the target is ours to choose (in mocap coordinates).
% target_position = [0;0.5];
target_position = [-0.5;1];

% Sanity-check the target against the arena before running: put it somewhere the
% bots can actually reach, well inside the tracked volume.
arena_limits = [-1.5 1.5 -1.5 1.5];        % [xmin xmax ymin ymax] (m); [] to disable the geo-fence

env = env_hardware(pursuer_cfg, evader_cfgs, target_position, ...
    'timestep',   0.05, ...        % control period (s)
    'tolerance',  0.5, ...        % capture radius (m) -- big bots, also avoids collisions
    'max_speed',  0.18, ...        % m/s for BOTH pursuer and evaders (keeps alpha = 1)
    'policy',     "closest_next_step", ...
    'arena_limits', arena_limits, ...
    'reset',      false);          % true forces fresh EV3/ROS connections

% Options for policy: "closest_next_step", "standard", "squaresum",
%                     "squaresump", "heuristic", "closest"

%% ================= RUN =================
% run() is a METHOD on purpose: its onCleanup stops every motor on every bot on
% a normal finish, on an error, and on Ctrl-C. Never drive the bots from a
% script-level loop -- onCleanup does not fire in a script.
history = env.run(2000, 120);      % max_steps, max_time (s)

env.plot_history(history);

%% ================= SAVE HISTORY =================
% Persist the full run for later analysis / TikZ (pgfplots) plots. `history`
% holds every trajectory -- pursuer_positions (2 x T), evader_positions
% (2 x n x T), the target, timing, capture status and the outcome -- so this
% one file has all the agent paths. Saved as .mat (keeps the exact struct);
% export to columnar .dat when you build the plots.
history.max_speed = env.max_speed;         % record the run settings alongside
history.tolerance = env.capture_tolerance;
history.policy    = env.pursuer_policy;

if ~isfolder('results')
    mkdir('results');
end
stamp    = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
run_file = fullfile('results', "run_" + stamp + ".mat");
save(char(run_file), 'history');
fprintf("Saved run history to %s\n", run_file);
