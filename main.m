clc
close all

%% ================= INITIAL CONDITIONS =================
% Where the pursuer / evader / target start positions come from:
%   "manual" - hand-typed below
%   "random" - uniformly sampled within pos_bounds below
%   "mat"    - initial frame of a saved run in results/*.mat
%   "mocap"  - live pose from OptiTrack (bots placed on the floor now)
init_source = "mat";
mat_file = "results/sim1.mat";
n_evaders = 4;              % only used by "random"
pos_bounds = [-2 2 -2 2];   % [xmin xmax ymin ymax] sampling box, only used by "random"
% Radius bands (as a fraction of the box half-diagonal) that the pursuer /
% evaders are sampled at, measured from the target -- only used by "random".
% Pursuer closer to the target, evaders further away.
pursuer_dist_frac = [0    0.25];
evader_dist_frac  = [0.4  0.9];
store_history = false;

% System parameters
timestep = 0.05;
tolerance = 0.12; % pursuer-evader capture tolerance

switch init_source

    case "manual"
        % Exp1
        pursuer_position = [0;-1];
        evader_positions = [0,1.35;1,0];
        target_position = [0.31225;-0.05];
        % Other examples:
        % pursuer_position = [-0.865;-0.2946]; evader_positions = [-0.02,1.079;-0.939,-0.45]; target_position = [-1;0.8];
        % pursuer_position = [-0.57;0.597];   evader_positions = [-1.217,0.62;-0.603,-0.59]; target_position = [-1;1];

    case "random"
        xmin = pos_bounds(1); xmax = pos_bounds(2);
        ymin = pos_bounds(3); ymax = pos_bounds(4);
        half_diag = 0.5*norm([xmax-xmin, ymax-ymin]);

        target_position = [xmin + (xmax-xmin)*rand; ymin + (ymax-ymin)*rand];

        % Sample a position at a random distance (within a radius band, as a
        % fraction of the box half-diagonal) and random angle from the
        % target, then clamp back into pos_bounds.
        sample_near_target = @(dist_frac) min(max( ...
            target_position + ...
            (dist_frac(1) + diff(dist_frac)*rand)*half_diag*[cos(2*pi*rand); sin(2*pi*rand)], ...
            [xmin; ymin]), [xmax; ymax]);

        pursuer_position = sample_near_target(pursuer_dist_frac);
        evader_positions = zeros(2, n_evaders);
        for i = 1:n_evaders
            evader_positions(:,i) = sample_near_target(evader_dist_frac);
        end

    case "mat"
        % Replay the START of a saved hardware run in simulation.
        S = load(mat_file);
        h = S.history;
        pursuer_position = h.pursuer_positions(:,1);
        n_ev = size(h.evader_positions, 2);
        evader_positions = zeros(2, n_ev);
        for i = 1:n_ev
            evader_positions(:,i) = h.evader_positions(:,i,1);
        end
        target_position = h.target_position(:);

    case "mocap"
        % Live snapshot from OptiTrack. Needs the vrpn bridge running and the
        % rigid bodies tracked. The target has no rigid body -> set it here.
        evader_names    = ["evader1", "evader2"];

        node = getMocapNode();
        P = readMocapPoses(node, ["pursuer", evader_names]);
        pursuer_position = P(:,1);
        evader_positions = P(:,2:end);
        target_position = [-0.5;1];

    otherwise
        error("main:badSource", "init_source must be 'manual', 'random', 'mat', or 'mocap'.");
end

n = size(evader_positions, 2); % number of evaders

fprintf("Init source: %s | %d evader(s)\n", init_source, n);
fprintf("  pursuer (%+.3f, %+.3f)\n", pursuer_position(1), pursuer_position(2));
for i = 1:n
    fprintf("  evader%d (%+.3f, %+.3f)\n", i, evader_positions(1,i), evader_positions(2,i));
end
fprintf("  target  (%+.3f, %+.3f)\n", target_position(1), target_position(2));

env = Environment(n,timestep,tolerance,pursuer_position,evader_positions,target_position);

%%

pursuer_policy = "closest_next_step";
% Options: "closest_next_step", "standard", "squaresum", "squaresump", "heuristic", "closest"
env.set_pursuer_policy(pursuer_policy);

evader_policy = "straight_to_target";
% Options: "optimal" (win/alpha-dependent optimal evasion), "straight_to_target"
env.set_evader_policy(evader_policy);
if evader_policy == "straigth_to_target"
    tolerance = 0.21;
end

[done, history] = env.simulate(10000, true);

%% ================= SAVE HISTORY =================
% Persist the full run for later analysis / TikZ (pgfplots) plots, same
% convention as main_hardware.m. `history` holds every trajectory --
% pursuer_positions (2 x T), evader_positions (2 x n x T), the target,
% timing, capture status and the outcome -- so this one file has all the
% agent paths. Saved as .mat (keeps the exact struct); export to columnar
% .dat when you build the plots.
if store_history
    history.max_speed = env.pursuer_speed;     % record the run settings alongside
    history.tolerance = env.capture_tolerance;
    history.policy    = env.pursuer_policy;
    history.evader_policy = env.evader_policy;
    
    if ~isfolder('results')
        mkdir('results');
    end
    stamp    = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    run_file = fullfile('results', "run_" + stamp + ".mat");
    save(char(run_file), 'history');
    fprintf("Saved run history to %s\n", run_file);
end