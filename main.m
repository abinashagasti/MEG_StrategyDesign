clc
close all

%% ================= INITIAL CONDITIONS =================
% Where the pursuer / evader / target start positions come from:
%   "manual" - hand-typed below
%   "mat"    - initial frame of a saved run in results/*.mat
%   "mocap"  - live pose from OptiTrack (bots placed on the floor now)
init_source = "mocap";
mat_file = "results/Exp2_1.mat";

switch init_source

    case "manual"
        % Exp1
        pursuer_position = [-0.8984; 0.3425];
        evader_positions = [-0.6616, 0.9380; -0.9060, -0.9713];
        target_position  = [0; 0.5];
        % Other examples:
        % pursuer_position = [-0.865;-0.2946]; evader_positions = [-0.02,1.079;-0.939,-0.45]; target_position = [-1;0.8];
        % pursuer_position = [-0.57;0.597];   evader_positions = [-1.217,0.62;-0.603,-0.59]; target_position = [-1;1];

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
        error("main:badSource", "init_source must be 'manual', 'mat', or 'mocap'.");
end

n = size(evader_positions, 2); % number of evaders

fprintf("Init source: %s | %d evader(s)\n", init_source, n);
fprintf("  pursuer (%+.3f, %+.3f)\n", pursuer_position(1), pursuer_position(2));
for i = 1:n
    fprintf("  evader%d (%+.3f, %+.3f)\n", i, evader_positions(1,i), evader_positions(2,i));
end
fprintf("  target  (%+.3f, %+.3f)\n", target_position(1), target_position(2));

% System parameters
timestep = 0.05;
tolerance = 0.35;

env = Environment(n,timestep,tolerance,pursuer_position,evader_positions,target_position);

%%

pursuer_policy = "closest_next_step";
% Options: "closest_next_step", "standard", "squaresum", "squaresump", "heuristic", "closest"
env.set_pursuer_policy(pursuer_policy);

[done, history] = env.simulate(10000, true);