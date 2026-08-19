nvars = 5;   % x1,y1,x2,y2,s
objective = @(z) z(5);
pursuer_position = [1;1];
target_position = [0;0];
timestep = 0.05;
nonlcon   = @(z) deal(flip_feasibility_constraints(z(1:4), pursuer_position, ...
    target_position, timestep) - z(5), []);

xmin=-2;xmax=2;
ymin=-2;ymax=2;

lb = [xmin; ymin; xmin; ymin; -10];
ub = [xmax; ymax; xmax; ymax;  10];
options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'off');

num_starts = 1000;

% Preallocated, per-trial ("sliced") storage -- required for parfor,
% since appending to a shared growing array (hits = [hits, ...]) can't
% be coordinated across workers running out of order.
hit_positions = NaN(4, num_starts);
hit_margins   = NaN(1, num_starts);
hit_exitflags = NaN(1, num_starts);

fprintf("Searching for feasible (evader1, evader2) positions -- pursuer_position = [%.3f, %.3f], target_position = [%.3f, %.3f], timestep = %.3f, %d start(s)\n\n", ...
    pursuer_position(1), pursuer_position(2), target_position(1), target_position(2), timestep, num_starts);

parfor trial = 1:num_starts
    z0 = [xmin + (xmax-xmin)*rand(4,1); 1];   % random evader guess, s starts positive (infeasible)
    [z, fval, exitflag] = fmincon(objective, z0, [], [], [], [], lb, ub, nonlcon, options);

    hit_exitflags(trial) = exitflag;
    hit_margins(trial)   = fval;
    if exitflag > 0 && fval < -1e-3
        hit_positions(:, trial) = z(1:4);
        fprintf("Trial %d: feasible (margin %.5f)\n", trial, fval);
    else
        fprintf("Trial %d: no feasible point (exitflag %d, margin %.5f)\n", trial, exitflag, fval);
    end
end

% Post-process sequentially -- cheap (only run for actual hits) and lets
% us print full diagnostics in trial order rather than the interleaved
% order the parfor loop above produced.
feasible_trials = find(hit_exitflags > 0 & hit_margins < -1e-3);
n_feasible = numel(feasible_trials);

fprintf("\n--- %d feasible initial condition(s) out of %d start(s) ---\n\n", n_feasible, num_starts);

for k = 1:n_feasible
    trial = feasible_trials(k);
    v = hit_positions(:, trial);
    evader1_position = v(1:2);
    evader2_position = v(3:4);

    [~, info] = check_one_step_flip(pursuer_position, [evader1_position, evader2_position], target_position, timestep);

    fprintf("Hit #%d (trial %d, margin %.5f):\n", k, trial, hit_margins(trial));
    fprintf("  evader1_position = [%.5f; %.5f]\n", evader1_position(1), evader1_position(2));
    fprintf("  evader2_position = [%.5f; %.5f]\n", evader2_position(1), evader2_position(2));
    fprintf("  chosen = evader%d, deviant = evader%d\n", info.chosen, info.deviant);
    fprintf("  at k:   dPT0 = %.5f, dPE0 = [%.5f, %.5f], dET0 = [%.5f, %.5f]\n", ...
        norm(pursuer_position - target_position), info.dPE0(1), info.dPE0(2), info.dET0(1), info.dET0(2));
    fprintf("  at k+1: dPT1 = %.5f, dEdT1_dev = %.5f, dist_next_opt(chosen) = %.5f\n\n", ...
        info.dPT1, info.dEdT1_dev, info.dist_next_opt(info.chosen));
end

hits = hit_positions(:, feasible_trials);
