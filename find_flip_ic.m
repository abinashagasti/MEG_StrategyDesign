function results = find_flip_ic(opts)
% FIND_FLIP_IC  Grid-search BOTH evader1's and evader2's positions for a
% fixed pursuer and target, looking for positions where the PDR -> EDR
% flip check (check_one_step_flip.m) is satisfied. Parallelized over
% evader1's grid via parfor.
%
%   results = find_flip_ic()
%   results = find_flip_ic(pursuer_position=[0;-1], target_position=[0;0], ...
%                           evader1_bounds=[-2 2 -2 2], evader1_resolution=0.1, ...
%                           evader2_bounds=[-2 2 -2 2], evader2_resolution=0.1)
%
% pursuer_position and target_position are held fixed. evader1_position
% and evader2_position are EACH swept over their own grid (evaderK_bounds,
% a [xmin xmax ymin ymax] box, at spacing evaderK_resolution), so this is
% a 4D search over (evader1_position, evader2_position) pairs. For every
% pair:
%   (i)  condition (i) is checked directly for both evaders: the target
%        must be strictly inside the pursuer dominance region against
%        each one (||x_P-x_T|| < ||x_1-x_T|| and ||x_P-x_T|| < ||x_2-x_T||).
%        The evader1 check is applied per outer (parfor) iteration so
%        evader2's inner grid is skipped entirely whenever evader1 alone
%        already fails it.
%   (ii) check_one_step_flip is called on [evader1_position, evader2_position]
%        to test the PDR -> EDR flip conditions there. A pair is kept
%        whenever it returns flip=true, regardless of which evader ends
%        up "chosen" vs "deviant" -- check results(k).chosen /
%        results(k).check_info.deviant to see which one it was for a
%        given hit.
%
% No full Environment.simulate() run is performed here -- this is purely
% the one-step algebraic check, swept over both evaders' positions.
% Validate any hit yourself afterwards, e.g.:
%   env = Environment(2, timestep, 0.2, pursuer_position, [evader1_position, evader2_position], target_position);
%   env.set_pursuer_policy("closest_next_step");
%   env.set_evader_policy("straight_to_target");
%   [done, history] = env.simulate(3000, true);
%   history.termination_reason
%
% PARALLELIZATION: the outer sweep over evader1's grid runs as a parfor
% -- each worker owns one evader1 grid point and runs the full evader2
% inner double-loop serially for it, collecting its own local hits into
% one cell per iteration; everything is concatenated after the parfor
% finishes. This needs no pool-specific setup on your part: if a
% parallel pool is already open (e.g. via `parpool`), parfor uses it
% automatically; if no pool is open, MATLAB just runs the same code
% serially (unless your parallel preferences disable that fallback).
%
% Because parfor iterations can't communicate with each other, two
% things work differently from a plain for-loop version:
%   - max_results no longer stops the search early -- every pair still
%     gets tested, and results are TRIMMED to max_results afterwards.
%   - progress printing (verbose) is interleaved/out of order across
%     workers, since iterations don't run in a guaranteed sequence.
%
% Grid cost: with default resolution 0.1 over a [-2,2]x[-2,2] box that's
% 41 x 41 = 1681 points per evader, i.e. up to ~1681^2 ~= 2.8M pairs
% tested before condition (i) filtering (fewer in practice once evader1's
% filter and evader2's inner filter cut in). Tighten evader1_resolution /
% evader2_resolution / the *_bounds if that is too slow even in parallel.
%
% Returns a struct array `results`, one entry per matching pair:
%   evader1_position, evader2_position,
%   chosen (1 or 2, whichever check_one_step_flip picked),
%   check_info (the full info struct from check_one_step_flip, including
%   .deviant)

    arguments
        opts.pursuer_position   (2,1) double  = [0; -1]
        opts.target_position    (2,1) double  = [0; 0]
        opts.evader1_bounds     (1,4) double  = [-2 2 -2 2]   % [xmin xmax ymin ymax] for evader1
        opts.evader1_resolution (1,1) double  = 0.01
        opts.evader2_bounds     (1,4) double  = [-2 2 -2 2]   % [xmin xmax ymin ymax] for evader2
        opts.evader2_resolution (1,1) double  = 0.01
        opts.timestep           (1,1) double  = 0.05
        opts.max_results        (1,1) double  = Inf           % results are TRIMMED to this many after the full parfor completes
        opts.verbose            (1,1) logical = true
    end

    pursuer_position = opts.pursuer_position;
    target_position  = opts.target_position;

    dPT0 = norm(pursuer_position - target_position);

    xs1 = opts.evader1_bounds(1):opts.evader1_resolution:opts.evader1_bounds(2);
    ys1 = opts.evader1_bounds(3):opts.evader1_resolution:opts.evader1_bounds(4);
    xs2 = opts.evader2_bounds(1):opts.evader2_resolution:opts.evader2_bounds(2);
    ys2 = opts.evader2_bounds(3):opts.evader2_resolution:opts.evader2_bounds(4);

    % Flatten evader1's grid into a single list so the parfor loop
    % variable is a plain 1:N1 range (parfor requires this).
    [X1, Y1] = ndgrid(xs1, ys1);
    evader1_list = [X1(:)'; Y1(:)'];   % 2 x N1
    N1 = size(evader1_list, 2);
    N2 = numel(xs2) * numel(ys2);

    if opts.verbose
        fprintf("evader1 grid: %d x %d = %d points. evader2 grid: %d x %d = %d points. Up to %d pairs before condition (i) filtering.\n", ...
            numel(xs1), numel(ys1), N1, numel(xs2), numel(ys2), N2, N1*N2);
    end

    timestep = opts.timestep;   % local copies for parfor broadcast
    verbose  = opts.verbose;

    results_cell   = cell(N1, 1);
    n_tested_vec   = zeros(N1, 1);

    parfor idx1 = 1:N1
        evader1_position = evader1_list(:, idx1);

        % (i) strict PDR for evader1 -- skip evader2's whole inner grid
        % if evader1 alone already fails it.
        dE1T0 = norm(evader1_position - target_position);
        if ~(dPT0 < dE1T0)
            results_cell{idx1} = struct('evader1_position', {}, 'evader2_position', {}, 'chosen', {}, 'check_info', {});
            continue
        end

        local_hits = struct('evader1_position', {}, 'evader2_position', {}, 'chosen', {}, 'check_info', {});
        local_tested = 0;

        for x2i = 1:numel(xs2)
            for y2i = 1:numel(ys2)
                evader2_position = [xs2(x2i); ys2(y2i)];
                local_tested = local_tested + 1;

                % (i) strict PDR for evader2
                dE2T0 = norm(evader2_position - target_position);
                if ~(dPT0 < dE2T0)
                    continue
                end

                evader_positions = [evader1_position, evader2_position];
                [flip, info] = check_one_step_flip(pursuer_position, evader_positions, target_position, timestep);
                if ~flip
                    continue
                end

                hit = struct('evader1_position', evader1_position, ...
                              'evader2_position', evader2_position, ...
                              'chosen', info.chosen, ...
                              'check_info', info);
                local_hits(end+1) = hit; %#ok<AGROW>

                if verbose
                    fprintf("Hit: evader1_position = [%.3f, %.3f], evader2_position = [%.3f, %.3f]  (chosen=evader%d, deviant=evader%d)\n", ...
                        evader1_position(1), evader1_position(2), ...
                        evader2_position(1), evader2_position(2), info.chosen, info.deviant);
                end
            end
        end

        results_cell{idx1} = local_hits;
        n_tested_vec(idx1) = local_tested;
    end

    results = [results_cell{:}];
    n_tested = sum(n_tested_vec);
    n_found = numel(results);

    if n_found > opts.max_results
        results = results(1:opts.max_results);
        fprintf("Tested %d pairs, found %d matching (evader1, evader2) position(s), trimmed to max_results=%d.\n", ...
            n_tested, n_found, opts.max_results);
    else
        fprintf("Tested %d pairs, found %d matching (evader1, evader2) position(s).\n", n_tested, n_found);
    end
end
