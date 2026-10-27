% SIM_AUDIT  Read-only audit of the stored Section V-B simulations (items 1-13)
% plus reruns at dt = 0.025, rho = 0.12 (item 14) and a feint-evader test of
% the robust controller (item 15).
%
% Uses the repo's Environment / Pursuer / Evader classes UNCHANGED. The feint
% run needs an evader policy the code does not have, so feint_run() below
% reproduces Environment.step line-for-line and swaps only the evader velocity.
%
% Index conventions of Environment.simulate (Environment.m:223-273, 319-328):
%   step s uses positions(:,s) -> updateTermination -> invasion check ->
%   headings -> move; result stored at s+1, captured flags stored at s+1.
%   So captured_evaders(:,s+1) is the status decided FROM positions(:,s),
%   and the active set used for the headings of step s is ~captured(:,s+1).

clear; clc; close all
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', '..'));                 % repo root: strategy classes
figdir = fullfile(here, 'figs'); if ~isfolder(figdir), mkdir(figdir); end
out = fopen(fullfile(here, 'sim_results.txt'), 'w');
log = @(varargin) [fprintf(varargin{:}), fprintf(out, varargin{:})];

base = '/Users/abinashagasti/IITM/Research/Paper Submissions/Multiplayer Evader/Modified Experiment Images/';
ex(1) = struct('name', "Example 3", 'file', [base 'Simulation 1/sim1_file.mat'], ...
               'paper', [0.85 1.32; 2.0 1.095; 3.7 1.265], 'order', [3 2 1], 'policy', "optimal");
ex(2) = struct('name', "Example 4", 'file', [base 'Simulation 4/sim1_evader_deviation.mat'], ...
               'paper', [0.8 1.318; 1.6 1.151; 2.9 1.903], 'order', [3 2 1], 'policy', "straight_to_target");
ex(3) = struct('name', "Example 5", 'file', [base 'Simulation 2/evader_sequence_switch_file.mat'], ...
               'paper', [0.55 0.421; 1.0 0.538; 1.6 0.845; 2.35 0.108], 'order', [2 1 3 4], 'policy', "optimal");
extra = struct('name', "Simulation 3 (not in V-B)", 'file', [base 'Simulation 3/sim2_evader_deviation.mat']);

%% ================= Part C: stored runs =================
R = cell(1, 3);
for x = 1:3
    h = load(ex(x).file).history;
    dt = h.time(2) - h.time(1);
    log("\n==================== %s  [%s]\n", ex(x).name, ex(x).file);
    log("stored: tolerance %.3g, dt %.3g, policy %s, evader_policy %s, max_speed %g, steps %d, reason %s\n", ...
        h.tolerance, dt, h.policy, field_or(h, 'evader_policy', "<absent>"), h.max_speed, h.step_count, h.termination_reason);
    R{x} = analyze(h, dt, log);
    compare_paper(R{x}, ex(x), log);

    % reproducibility: rerun with HEAD code from the stored initial state
    P0 = h.pursuer_positions(:,1); E0 = squeeze(h.evader_positions(:,:,1)); T = h.target_position;
    env = Environment(size(E0,2), dt, h.tolerance, P0, E0, T);
    env.set_pursuer_policy("closest_next_step"); env.set_evader_policy(ex(x).policy);
    [~, h2] = evalc_sim(env);
    same = isequal(size(h2.pursuer_positions), size(h.pursuer_positions));
    if same
        dmax = max([abs(h2.pursuer_positions(:) - h.pursuer_positions(:)); abs(h2.evader_positions(:) - h.evader_positions(:))]);
        log("[repro] HEAD code rerun: same length (%d samples), max |position difference| = %.3g, captures identical: %d\n", ...
            numel(h2.time), dmax, isequal(h2.captured_evaders, h.captured_evaders));
    else
        log("[repro] HEAD code rerun: DIFFERENT length (%d vs %d samples), reason %s\n", numel(h2.time), numel(h.time), h2.termination_reason);
    end
end
log("\n[13] Example 4 vs Example 3 distance to target at capture (stored):\n");
for i = 1:3
    log("     E%d: Ex4 %.4f vs Ex3 %.4f -> farther in Ex4: %d\n", i, R{2}.capDistT(i), R{1}.capDistT(i), R{2}.capDistT(i) > R{1}.capDistT(i));
end

h = load(extra.file).history;
log("\n==================== %s  [%s]\n", extra.name, extra.file);
log("stored: tolerance %.3g, dt %.3g, evader_policy %s, T=(%.3f,%.3f), P=(%.3f,%.3f), %d evaders\n", h.tolerance, ...
    h.time(2)-h.time(1), field_or(h,'evader_policy',"<absent>"), h.target_position, h.pursuer_positions(:,1), size(h.evader_positions,2));
analyze(h, h.time(2)-h.time(1), log);

%% ================= Part D, item 14: dt = 0.025, rho = 0.12 =================
dtN = 0.025; rhoN = 0.12; RN = cell(1,3);
for x = 1:3
    h = load(ex(x).file).history;
    P0 = h.pursuer_positions(:,1); E0 = squeeze(h.evader_positions(:,:,1)); T = h.target_position;
    env = Environment(size(E0,2), dtN, rhoN, P0, E0, T);
    env.set_pursuer_policy("closest_next_step"); env.set_evader_policy(ex(x).policy);
    [~, hN] = evalc_sim(env);
    hN.tolerance = rhoN;                       % simulate() does not store it (main.m adds it on save)
    log("\n==================== RERUN %s  dt=%.3f rho=%.2f evader_policy=%s (HEAD code; invasion radius 1.1*dt=%.4f)\n", ...
        ex(x).name, dtN, rhoN, ex(x).policy, env.target_capture_tolerance);
    log("reason %s, steps %d\n", hN.termination_reason, hN.step_count);
    RN{x} = analyze(hN, dtN, log);
    compare_paper(RN{x}, ex(x), log);
    save(fullfile(here, "rerun_" + strrep(lower(ex(x).name), " ", "") + "_dt0025.mat"), 'hN');
    plot_run(hN, sprintf('%s rerun, \\Deltat=0.025, \\rho=0.12, evaders: %s', ex(x).name, ex(x).policy), ...
             fullfile(figdir, "rerun_" + strrep(lower(ex(x).name), " ", "") + "_dt0025.png"));
end
log("\n[14/13] Rerun Example 4 vs Example 3 distance to target at capture:\n");
for i = 1:3
    log("     E%d: Ex4 %.4f vs Ex3 %.4f -> farther in Ex4: %d\n", i, RN{2}.capDistT(i), RN{1}.capDistT(i), RN{2}.capDistT(i) > RN{1}.capDistT(i));
end

%% ================= Part D, item 15: feint evaders, robust controller, 40 s =================
h = load(ex(2).file).history;
for dtF = [0.025 0.05]
    hF = feint_run(h.pursuer_positions(:,1), squeeze(h.evader_positions(:,:,1)), h.target_position, dtF, 0.12, 0.8, 40);
    log("\n==================== FEINT (Example 4 ICs) dt=%.3f rho=0.12 flee radius 0.8, 40 s\n", dtF);
    log("reason %s at t=%.3f s, steps %d; invasion rule used: |E-T| < %.4f and |P-T| > %.4f (Environment.m:243)\n", ...
        hF.termination_reason, hF.time(end), hF.step_count, hF.inv_radius, 2*hF.inv_radius);
    RF = analyze(hF, dtF, log);
    log("[15] min distance of any active evader to T over the run: %.4f m; ever within dt=%.3f of T (pursuer condition ignored): %d\n", ...
        RF.minDistT, dtF, RF.minDistT < dtF);
    plot_run(hF, sprintf('Feint evaders (flee if |P-E|<0.8, else to T), robust P, \\Deltat=%.3f, \\rho=0.12', dtF), ...
             fullfile(figdir, sprintf('feint_example4_dt%s.png', strrep(sprintf('%.3f', dtF), '.', ''))));
    save(fullfile(here, sprintf('feint_example4_dt%s.mat', strrep(sprintf('%.3f', dtF), '.', ''))), 'hF');
end
fclose(out);

%% ======================================================================
function v = field_or(s, f, d)
    if isfield(s, f), v = s.(f); else, v = d; end
end

function [txt, h] = evalc_sim(env)
    % Environment.simulate prints mode changes; keep the console readable.
    txt = evalc('[~, h] = env.simulate(100000, false);');
end

function R = analyze(h, dt, log)
    wrap = @(a) angle(exp(1i*a));
    t = h.time; P = h.pursuer_positions; E = h.evader_positions; cap = h.captured_evaders;
    T = h.target_position(:); rho = h.tolerance; [~, n, N] = size(E);
    log("ICs: T=(%.3f,%.3f) P=(%.3f,%.3f)", T, P(:,1)); for i=1:n, log(" E%d=(%.3f,%.3f)", i, E(:,i,1)); end, log("\n");

    % ---- item 8: captures ----
    R.capDistT = nan(1,n); R.capT = nan(1,n); R.capFlagT = nan(1,n); js = nan(1,n);
    for i = 1:n
        d = vecnorm(P - squeeze(E(:,i,:)));
        j  = find(d <= rho, 1);  jl = find(d < rho, 1);  jf = find(cap(i,:), 1);
        if isempty(j)
            log("[8] E%d never within rho\n", i); continue
        end
        js(i) = j; R.capT(i) = t(j); R.capDistT(i) = norm(E(:,i,j) - T); R.capFlagT(i) = t(jf);
        log("[8] E%d: first sample |P-E|<=rho: t=%.3f (idx %d, |P-E|=%.4f; strict '<' same idx: %d) | code flag set at t=%.3f | |E-T|=%.4f\n", ...
            i, t(j), j, d(j), isequal(j, jl), t(jf), R.capDistT(i));
    end
    [~, ord] = sort(R.capT); R.order = ord(~isnan(R.capT(ord)));
    log("    capture order: %s\n", strjoin("E" + string(R.order), " -> "));

    % ---- items 9-12 over decision steps s (positions s, active = ~cap(:,s+1)) ----
    nfb = 0; fbT = []; minDT = inf; minDTat = NaN; sepMin = inf; sepAt = NaN; sepPair = [NaN NaN];
    crit = nan(1, N-1); oldInv = []; newInv = [];
    for s = 1:N-1
        act = find(~cap(:, s+1))';
        if isempty(act), continue, end
        Ps = P(:,s); Ea = squeeze(E(:,act,s)); if numel(act)==1, Ea = Ea(:); end
        dT = vecnorm(Ea - T); dPT = norm(Ps - T);
        [m, k] = min(dT); if m < minDT, minDT = m; minDTat = t(s); minWho = act(k); end
        if any(dPT > dT), nfb = nfb + 1; fbT(end+1) = t(s); end %#ok<AGROW>
        % invasion rules (old: radius rho; HEAD: radius 1.1*dt), each with |P-T| > 2*radius
        if any(dT < rho & dPT > 2*rho), oldInv(end+1) = t(s); end %#ok<AGROW>
        if any(dT < 1.1*dt & dPT > 2.2*dt), newInv(end+1) = t(s); end %#ok<AGROW>
        pur = Pursuer(Ps, 1, 0);
        th = nan(1, numel(act)); best = inf;
        for q = 1:numel(act)
            [ps, th(q)] = pur.optimal_headings_Ei(Ea(:,q), T);
            dn = norm(Ea(:,q) + dt*[cos(ps); sin(ps)] - T);      % Pursuer.m:75-79, lookahead = timestep
            if dn < best, best = dn; crit(s) = act(q); end
        end
        if any(dPT > dT), crit(s) = 0; end                        % fallback step: no critical evader (0)
        for a = 1:numel(act), for c = a+1:numel(act)
            sp = abs(wrap(th(a) - th(c)));
            if sp < sepMin, sepMin = sp; sepAt = t(s); sepPair = act([a c]); end
        end, end
    end
    R.minDistT = minDT; R.nFallback = nfb;
    log("[9] steps with target NOT in pursuer dominance region (fallback invoked): %d", nfb);
    if nfb, log(" at t = %s", sprintf('%.3f ', fbT)); end, log("\n");
    log("[10] min |E-T| over active evaders: %.4f m (E%d at t=%.3f)\n", minDT, minWho, minDTat);
    if isfinite(sepMin)
        log("[11] min pairwise separation of theta*_i (active): %.2f deg between E%d and E%d at t=%.3f\n", rad2deg(sepMin), sepPair, sepAt);
    else
        log("[11] never two active evaders\n");
    end
    % ---- item 12: critical-evader sequence ----
    seq = ""; prev = NaN;
    for s = 1:N-1
        c = crit(s); if isnan(c), continue, end
        if ~isequal(c, prev)
            why = "";
            if ~isnan(prev) && prev > 0 && cap(prev, s+1), why = " (prev captured)"; elseif ~isnan(prev), why = " (threat switch)"; end
            lbl = "E" + c; if c == 0, lbl = "FALLBACK(T)"; end
            seq = seq + sprintf("t=%.3f: %s%s; ", t(s), lbl, why);
            prev = c;
        end
    end
    log("[12] critical evader sequence: %s\n", seq);
    log("[5] invasion check would fire with OLD rule (radius rho=%.3g): %s | HEAD rule (radius 1.1dt=%.4f): %s\n", rho, ...
        tern(isempty(oldInv), "never", "at t=" + sprintf('%.3f ', oldInv)), 1.1*dt, tern(isempty(newInv), "never", "at t=" + sprintf('%.3f ', newInv)));
    R.crit = crit; R.t = t;
end

function compare_paper(R, ex, log)
    log("[cmp] %s vs paper (paper order %s):\n", ex.name, strjoin("E" + string(ex.order), "->"));
    for q = 1:numel(ex.order)
        i = ex.order(q);
        log("     E%d: time %.3f (paper %.3f, diff %+.3f) | |E-T| %.4f (paper %.3f, diff %+.4f)\n", i, R.capT(i), ex.paper(q,1), ...
            R.capT(i) - ex.paper(q,1), R.capDistT(i), ex.paper(q,2), R.capDistT(i) - ex.paper(q,2));
    end
end

function s = tern(c, a, b)
    if c, s = a; else, s = b; end
end

function h = feint_run(P0, E0, T, dt, rho, flee_r, tmax)
    % Environment.step (Environment.m:223-273) with the evader velocity replaced:
    % flee straight away from P if |P-E| < flee_r, else head straight to T.
    n = size(E0, 2);
    env = Environment(n, dt, rho, P0, E0, T);
    env.set_pursuer_policy("closest_next_step");
    K = round(tmax/dt);
    h.time = (0:K)*dt; h.pursuer_positions = nan(2, K+1); h.evader_positions = nan(2, n, K+1);
    h.captured_evaders = false(n, K+1); h.target_position = T; h.tolerance = rho;
    h.pursuer_positions(:,1) = P0; h.evader_positions(:,:,1) = E0;
    reason = "timeout_40s"; k = 0;
    while k < K
        env.updateTermination();
        act = find(~env.captured_evaders);
        if isempty(act), reason = "all_evaders_captured"; end
        for i = act
            if norm(env.evaders(i).position - T) < env.target_capture_tolerance && ...
               norm(env.pursuer.position - T) > 2*env.target_capture_tolerance
                reason = "evader_reached_target"; break
            end
        end
        if reason ~= "timeout_40s", k = k + 1; h = rec(h, env, k); break, end
        win = env.check_initialization(env.evaders(act), false);
        Ea = env.return_evader_positions(env.evaders(act));
        pol = env.pursuer_policy; pur = env.pursuer; %#ok<NASGU>
        evalc('pv = pur.heading_velocity(Ea, T, dt, win, pol);');   % silence fallback print
        Pnow = env.pursuer.position;                                 % evaders react to pre-move P, as in step()
        env.pursuer.updatePos(Pnow + dt*pv);
        for i = act
            e = env.evaders(i).position;
            if norm(Pnow - e) < flee_r, v = (e - Pnow)/norm(e - Pnow); else, v = (T - e)/norm(T - e); end
            env.evaders(i).updatePos(e + dt*v);
        end
        k = k + 1; h = rec(h, env, k);
    end
    last = k + 1;
    h.time = h.time(1:last); h.pursuer_positions = h.pursuer_positions(:,1:last);
    h.evader_positions = h.evader_positions(:,:,1:last); h.captured_evaders = h.captured_evaders(:,1:last);
    h.step_count = k; h.termination_reason = reason; h.inv_radius = env.target_capture_tolerance;
end

function h = rec(h, env, k)
    h.pursuer_positions(:,k+1) = env.pursuer.position;
    for i = 1:numel(env.evaders), h.evader_positions(:,i,k+1) = env.evaders(i).position; end
    h.captured_evaders(:,k+1) = env.captured_evaders';
end

function plot_run(h, ttl, file)
    f = figure('Visible', 'off', 'Position', [0 0 700 650]); hold on; axis equal; grid on
    T = h.target_position; n = size(h.evader_positions, 2); col = lines(n+1);
    plot(T(1), T(2), 'gp', 'MarkerSize', 14, 'MarkerFaceColor', 'g', 'DisplayName', 'T');
    plot(h.pursuer_positions(1,:), h.pursuer_positions(2,:), '-', 'Color', col(1,:), 'LineWidth', 1.5, 'DisplayName', 'P');
    plot(h.pursuer_positions(1,1), h.pursuer_positions(2,1), 'o', 'Color', col(1,:), 'HandleVisibility', 'off');
    for i = 1:n
        X = squeeze(h.evader_positions(:,i,:));
        plot(X(1,:), X(2,:), '-', 'Color', col(i+1,:), 'LineWidth', 1.2, 'DisplayName', sprintf('E%d', i));
        plot(X(1,1), X(2,1), 'o', 'Color', col(i+1,:), 'HandleVisibility', 'off');
        j = find(h.captured_evaders(i,:), 1);
        if ~isempty(j), plot(X(1,j), X(2,j), 'x', 'Color', col(i+1,:), 'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off'); end
    end
    legend('Location', 'bestoutside'); title(ttl); xlabel('x'); ylabel('y');
    exportgraphics(f, file, 'Resolution', 150); close(f);
end
