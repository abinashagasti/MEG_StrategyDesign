% AUDIT_EXPERIMENTS  Read-only analysis of the two 1v2 hardware runs (items 14-23).
%
% Loads the logged histories straight from the paper folder (never writes there),
% recomputes the commanded headings offline with the UNCHANGED strategy classes in
% the repo root, and writes numbers to results_<exp>.txt plus PNG plots into
% paper_audit/figs/.
%
% TIMING MODEL (from env_hardware.run/step, see audit notes):
%   history.time(j)  = toc AFTER iteration j-1 finished (includes pause(0.05)).
%   position(:,j)    = pose SENSED at the START of iteration j-1, i.e. right
%                      after time(j-1) was stamped.  So the physical measurement
%                      time of sample j (j>=2) is t_sense(j) ~= time(j-1) (+ one
%                      receive() wait).  Sample 1 is the constructor read (before
%                      t0) and sample 2 is sensed at ~t=0 before any drive().
%   Command computed from sample j is applied during [t_sense(j), t_sense(j+1)],
%   so it is compared with the displacement X(:,j+1)-X(:,j).

clear; clc; close all
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..'));                 % Pursuer, Evader (read-only use)
figdir = fullfile(here, 'figs'); if ~isfolder(figdir), mkdir(figdir); end

base = '/Users/abinashagasti/IITM/Research/Paper Submissions/Multiplayer Evader/Modified Experiment Images/';
exps = struct('name', {'exp1','exp2'}, ...
              'file', {[base 'Experiment 1/Exp1_3.mat'], [base 'Experiment 2/Exp2_1.mat']});

LOOKAHEAD = 0.18 * 0.05;     % env_hardware.m:226  max_speed*timestep (main_hardware.m:39,41)
ALIGN_TOL = deg2rad(10);      % item 20 threshold
wrap = @(a) angle(exp(1i*a));

for e = exps
    S = load(e.file); h = S.history;
    out = fopen(fullfile(here, "results_" + e.name + ".txt"), 'w');
    pr = @(varargin) fprintf(out, varargin{:});
    both = @(varargin) [fprintf(varargin{:}), fprintf(out, varargin{:})];

    t   = h.time(:)';                       N = numel(t);
    P   = h.pursuer_positions;               n = size(h.evader_positions, 2);
    E   = h.evader_positions;                T = h.target_position(:);
    cap = h.captured_evaders;                rho = h.tolerance;
    ts  = [NaN, t(1:end-1)];                 % t_sense(j) = time(j-1)

    both("\n================ %s  (%s)\n", upper(e.name), e.file);
    both("N=%d samples, time span %.4f s, max_speed %.2f, tolerance %.2f, policy %s, reason %s\n", ...
         N, t(end), h.max_speed, rho, h.policy, h.termination_reason);
    both("Initial: T=(%.3f,%.3f) P=(%.3f,%.3f)", T, P(:,1));
    for i = 1:n, both(" E%d=(%.3f,%.3f)", i, E(:,i,1)); end, both("\n");

    %% ---------- item 15: achieved sampling period ----------
    dt = diff(t);
    both("\n[15] log/iteration period dt = diff(time): all %d: mean %.4f std %.4f min %.4f max %.4f\n", ...
         numel(dt), mean(dt), std(dt), min(dt), max(dt));
    both("     excluding first interval (motor start-up): mean %.4f std %.4f min %.4f max %.4f, median %.4f\n", ...
         mean(dt(2:end)), std(dt(2:end)), min(dt(2:end)), max(dt(2:end)), median(dt(2:end)));
    both("     dt list: %s\n", sprintf('%.3f ', dt));

    %% ---------- item 17/18: captures, closest approach ----------
    dPE = zeros(n, N); dTE = zeros(n, N);
    for i = 1:n
        dPE(i,:) = vecnorm(P - squeeze(E(:,i,:)));
        dTE(i,:) = vecnorm(squeeze(E(:,i,:)) - T);
    end
    dPT = vecnorm(P - T);
    capIdx = zeros(1, n);
    for i = 1:n
        j  = find(dPE(i,:) < rho, 1);           % Environment.m:217 uses strict '<'
        jf = find(cap(i,:), 1);                 % logged flag
        capIdx(i) = j;
        both("[17] E%d: first sample with |P-E|<%.2f: j=%d, logged flag first at j=%d\n", i, rho, j, jf);
        both("     time(j) = %.4f s (paper convention = log stamp)  |  measured at t_sense = time(j-1) = %.4f s\n", t(j), ts(j));
        both("     capture happened between measurements at %.4f and %.4f s (prev |P-E| = %.3f, at j: %.3f)\n", ...
             ts(j-1), ts(j), dPE(i,j-1), dPE(i,j));
        both("     |E-T| at j = %.4f m, |P-T| at j = %.4f m\n", dTE(i,j), dPT(j));
        active = 1:j;                             % while still in the game
        [m, k] = min(dTE(i,active));
        both("[18] E%d closest approach to T while active: %.4f m at sample %d (time %.4f, t_sense %.4f); whole log min %.4f m\n", ...
             i, m, k, t(k), ts(k), min(dTE(i,:)));
    end

    %% ---------- item 19: dominance region ----------
    margin = inf(1, N);
    for j = 1:N
        act = find(~cap(:,j));
        if isempty(act), margin(j) = NaN; continue, end
        margin(j) = min(dTE(act,j)) - dPT(j);
    end
    [mm, km] = min(margin);
    both("\n[19] min over samples of (min_active |Ei-T| - |P-T|) = %.4f m at sample %d (t=%.3f). Fails at %d samples.\n", ...
         mm, km, t(km), sum(margin < 0));

    %% ---------- recompute commanded headings with repo strategy code ----------
    th_cmd   = nan(1, N);  crit = nan(1, N);  win_log = nan(1, N);
    psi_cmd  = nan(n, N);  th_star = nan(n, N);  sep = nan(1, N);
    tcomp    = nan(1, N);
    for j = 1:N
        act = find(~cap(:,j))';
        if isempty(act), continue, end          % all captured: no command (env_hardware.m:198)
        pur = Pursuer(P(:,j), 1, 0);
        evs = Evader.empty(1,0);
        for i = 1:n, evs(i) = Evader(E(:,i,j), 1, i); end %#ok<AGROW>
        Ea  = E(:,act,j);
        b   = min(vecnorm(T - Ea).^2 - norm(T - P(:,j))^2);   % Environment.barrier, alpha=1
        win = b >= 0;  win_log(j) = win;
        [~, th_cmd(j)] = pur.heading_velocity(Ea, T, LOOKAHEAD, win, "closest_next_step");
        tcomp(j) = timeit(@() step_compute(pur, evs, Ea, T, LOOKAHEAD, win, act));
        % critical evader (Pursuer.optimal_pursuer_heading lines 71-80)
        dbest = inf;
        for q = 1:numel(act)
            [ps, tq] = pur.optimal_headings_Ei(Ea(:,q), T);
            th_star(act(q), j) = tq;
            dn = norm(Ea(:,q) + LOOKAHEAD*[cos(ps); sin(ps)] - T);
            if dn < dbest, dbest = dn; crit(j) = act(q); end
        end
        if numel(act) >= 2
            s = inf;
            for a = 1:numel(act), for c = a+1:numel(act)
                s = min(s, abs(wrap(th_star(act(a),j) - th_star(act(c),j))));
            end, end
            sep(j) = s;
        end
        for i = act
            wi = (norm(T - E(:,i,j))^2 - norm(T - P(:,j))^2) >= 0;   % Environment.return_evader_velocities
            [~, psi_cmd(i,j)] = evs(i).heading_velocity(P(:,j), T, wi, 1, "optimal");
        end
    end
    both("\n[12] pursuer mode per sample (1 = intercept / win, 0 = head-to-target fallback): %s\n", sprintf('%d', win_log(~isnan(win_log))));
    both("     critical evader per sample: %s\n", sprintf('%d', crit(~isnan(crit))));
    both("[22] min angular separation of theta*_i over samples with 2 active evaders: %.2f deg (sample %d); per-sample: %s\n", ...
         rad2deg(min(sep)), find(sep == min(sep), 1), sprintf('%.1f ', rad2deg(sep(~isnan(sep)))));
    both("[23] offline compute time per iteration (strategy only, this Mac, timeit): mean %.3g s, max %.3g s\n", ...
         mean(tcomp, 'omitnan'), max(tcomp));

    %% ---------- item 14: speeds ----------
    names = ["P", "E" + (1:n)];
    POS   = cell(1, 1+n); POS{1} = P; for i = 1:n, POS{1+i} = squeeze(E(:,i,:)); end
    % Outlier rule (data-driven, applied to speed/heading stats only): a sample
    % whose implied speed to BOTH neighbours exceeds 3x the commanded 0.18 m/s
    % is a mocap glitch (one-frame jump and back) and is set to NaN.
    for a = 1:1+n
        X = POS{a};
        for j = 3:N-1      % interval j-1 -> j spans [time(j-2), time(j-1)]
            vin  = norm(X(:,j) - X(:,j-1)) / (t(j-1) - t(j-2));
            vout = norm(X(:,j+1) - X(:,j)) / (t(j) - t(j-1));
            if vin > 0.54 && vout > 0.54
                both("[outlier] %s sample %d (time %.4f) flagged: jump in %.3f m, out %.3f m -> excluded from speed/heading stats\n", ...
                     names(a), j, t(j), norm(X(:,j) - X(:,j-1)), norm(X(:,j+1) - X(:,j)));
                X(:,j) = NaN;
            end
        end
        POS{a} = X;
    end
    stopIdx = [N, capIdx];                        % last sample each agent is driven
    capT = ts(capIdx);                            % physical capture-detection times
    vstat = nan(1+n, 2); vstatNaive = nan(1+n, 2);
    SPD = cell(1, 1+n);
    for a = 1:1+n
        X  = POS{a};
        d  = vecnorm(diff(X, 1, 2));              % displacement sample j -> j+1
        % proper convention: interval [t_sense(j), t_sense(j+1)] = [time(j-1), time(j)]
        v   = nan(1, N-1); vn = d ./ diff(t);
        for j = 2:N-1, v(j) = d(j) / (t(j) - t(j-1)); end
        tmid = nan(1, N-1); for j = 2:N-1, tmid(j) = 0.5*(t(j-1) + t(j)); end
        vs = movmedian(v, 3, 'omitnan');
        SPD{a} = struct('tmid', tmid, 'v', v, 'vs', vs, 'vnaive', vn);
        % steady window: after 1 s, before this agent stops, not within 1 s after any capture
        keep = ~isnan(v) & tmid > 1.0 & (1:N-1) < stopIdx(a);
        if a == 1
            for c = capT, keep = keep & ~(tmid > c & tmid < c + 1.0); end
        end
        vstat(a,:) = [mean(vs(keep)), std(vs(keep))];
        vstatNaive(a,:) = [mean(vn(keep)), std(vn(keep))];
        pathlen = sum(d(1:stopIdx(a)-1), "omitnan");
        both("[14] %s: steady smoothed speed mean %.4f std %.4f m/s over %d intervals (raw mean %.4f); naive-stamp mean %.4f; path %.3f m over driven span\n", ...
             names(a), vstat(a,1), vstat(a,2), sum(keep), mean(v(keep)), vstatNaive(a,1), pathlen);
        both("     raw per-interval speeds: %s\n", sprintf('%.3f ', v(2:end)));
    end
    for i = 1:n
        both("[14] speed ratio P/E%d = %.3f  (alpha_hat = E/P = %.3f)\n", i, vstat(1,1)/vstat(1+i,1), vstat(1+i,1)/vstat(1,1));
    end

    %% ---------- items 16, 20, 21: heading tracking ----------
    % motion direction of agent over interval j->j+1
    err0 = nan(1+n, N); err1 = nan(1+n, N);
    CMD = [th_cmd; psi_cmd];
    for a = 1:1+n
        X = POS{a};
        for j = 2:N-1
            dX = X(:,j+1) - X(:,j);
            if norm(dX) < 0.01, continue, end    % < 1 cm: direction is noise
            mdir = atan2(dX(2), dX(1));
            if ~isnan(CMD(a,j)),   err0(a,j) = abs(wrap(mdir - CMD(a,j)));   end   % lag 0
            if j >= 3 && ~isnan(CMD(a,j-1)), err1(a,j) = abs(wrap(mdir - CMD(a,j-1))); end % lag 1
        end
        e0 = err0(a, :); e1 = err1(a, :);
        both("[21] %s heading error |motion dir - cmd| (lag 0): mean %.1f deg, median %.1f, max %.1f (n=%d)\n", ...
             names(a), rad2deg(mean(e0,'omitnan')), rad2deg(median(e0,'omitnan')), rad2deg(max(e0)), sum(~isnan(e0)));
        both("[16] %s same vs PREVIOUS command (lag 1 sample): mean %.1f deg, median %.1f (n=%d)\n", ...
             names(a), rad2deg(mean(e1,'omitnan')), rad2deg(median(e1,'omitnan')), sum(~isnan(e1)));
    end
    both("     pursuer per-interval lag-0 errors (deg): %s\n", sprintf('%.0f ', rad2deg(err0(1,2:end-1))));

    % item 20: after first capture the pursuer's command switches to the other evader
    j0 = capIdx(1); if capIdx(2) < j0, j0 = capIdx(2); end
    both("\n[20] first capture detected from sample %d (t_sense %.3f s). Commanded pursuer heading before %.1f deg, after %.1f deg (change %.1f deg)\n", ...
         j0, ts(j0), rad2deg(th_cmd(j0-1)), rad2deg(th_cmd(j0)), rad2deg(abs(wrap(th_cmd(j0) - th_cmd(j0-1)))));
    X = P; aligned = NaN;
    for j = j0:N-1
        dX = X(:,j+1) - X(:,j); mdir = atan2(dX(2), dX(1));
        both("     interval %d->%d [%.3f, %.3f]s: motion %.1f deg, cmd %.1f deg, err %.1f deg, moved %.3f m\n", ...
             j, j+1, ts(j), ts(j+1), rad2deg(mdir), rad2deg(th_cmd(j)), rad2deg(abs(wrap(mdir - th_cmd(j)))), norm(dX));
        if isnan(aligned) && abs(wrap(mdir - th_cmd(j))) <= ALIGN_TOL, aligned = j; end
    end
    if isnan(aligned)
        both("     never aligned within 10 deg before the run ended\n");
    else
        both("     first interval within 10 deg: %d->%d, i.e. %.3f s after the capture measurement; pursuer travelled %.3f m before that interval\n", ...
             aligned, aligned+1, ts(aligned) - ts(j0), sum(vecnorm(diff(P(:, j0:aligned), 1, 2))));
    end
    fclose(out);

    %% ---------- plots ----------
    col = lines(1+n);
    f1 = figure('Visible','off','Position',[0 0 700 600]); hold on; axis equal; grid on
    hL = plot(T(1), T(2), 'gp', 'MarkerSize', 14, 'MarkerFaceColor', 'g', 'DisplayName', 'T (dotted: 0.2 m)');
    viscircles(T', 0.2, 'Color', [0 .6 0], 'LineStyle', ':');
    for a = 1:1+n
        X = POS{a}; last = stopIdx(a);
        hL(end+1) = plot(X(1,1:last), X(2,1:last), '.-', 'Color', col(a,:), 'MarkerSize', 12, 'DisplayName', names(a));
        q = 1:last; q = q(~isnan(CMD(a,q)));
        hL(end+1) = quiver(X(1,q), X(2,q), 0.06*cos(CMD(a,q)), 0.06*sin(CMD(a,q)), 0, 'Color', col(a,:)*0.6, 'LineWidth', 1, 'DisplayName', names(a)+" cmd heading");
    end
    for i = 1:n
        j = capIdx(i);
        plot([P(1,j) E(1,i,j)], [P(2,j) E(2,i,j)], 'k--');
        viscircles(P(:,j)', rho, 'Color', [.5 .5 .5], 'LineStyle', '--', 'LineWidth', 0.5);
    end
    legend(hL, 'Location', 'bestoutside');
    title(sprintf('%s: logged positions + recomputed commanded headings', upper(e.name)));
    xlabel('x (m)'); ylabel('y (m)');
    exportgraphics(f1, fullfile(figdir, e.name + "_trajectory_headings.png"), 'Resolution', 150);

    f2 = figure('Visible','off','Position',[0 0 800 400]); hold on; grid on
    for a = 1:1+n
        s = SPD{a}; m = (1:N-1) < stopIdx(a);
        plot(s.tmid(m), s.v(m), 'o', 'Color', col(a,:));
        plot(s.tmid(m), s.vs(m), '-', 'Color', col(a,:), 'LineWidth', 1.5);
    end
    yline(0.18, 'k:', 'commanded 0.18 m/s');
    for c = capT, xline(c, 'k--'); end
    xlabel('time (s, sensing-time convention)'); ylabel('speed (m/s)');
    legend(["P raw","P movmedian3","E1 raw","E1 movmedian3","E2 raw","E2 movmedian3"], 'Location', 'best');
    title(sprintf('%s: speed from consecutive positions', upper(e.name)));
    exportgraphics(f2, fullfile(figdir, e.name + "_speed.png"), 'Resolution', 150);

    f3 = figure('Visible','off','Position',[0 0 800 350]);
    stem(t(2:end), dt, 'filled'); grid on; yline(0.05, 'r:', 'code timestep 0.05 s'); yline(0.2, 'm:', 'paper \Deltat 0.2 s');
    xlabel('log time (s)'); ylabel('iteration period (s)'); title(sprintf('%s: achieved iteration period', upper(e.name)));
    exportgraphics(f3, fullfile(figdir, e.name + "_dt.png"), 'Resolution', 150);

    f4 = figure('Visible','off','Position',[0 0 800 450]); hold on; grid on
    for i = 1:n
        plot(t, dPE(i,:), '-o', 'Color', col(1+i,:), 'DisplayName', sprintf('|P-E%d|', i));
        plot(t, dTE(i,:), '--s', 'Color', col(1+i,:), 'DisplayName', sprintf('|E%d-T|', i));
    end
    plot(t, dPT, 'k-.', 'DisplayName', '|P-T|');
    plot(t, margin, 'm-', 'LineWidth', 1.5, 'DisplayName', 'dominance margin');
    yline(rho, 'k:', '\rho = 0.5'); yline(0, 'k-');
    xlabel('log time (s)'); ylabel('m'); legend('Location', 'best');
    title(sprintf('%s: distances and dominance margin (logged time stamps)', upper(e.name)));
    exportgraphics(f4, fullfile(figdir, e.name + "_distances_dominance.png"), 'Resolution', 150);

    f5 = figure('Visible','off','Position',[0 0 800 400]); hold on; grid on
    for a = 1:1+n
        plot(ts, rad2deg(err0(a,:)), '-o', 'Color', col(a,:), 'DisplayName', names(a) + " lag 0");
        plot(ts, rad2deg(err1(a,:)), ':x', 'Color', col(a,:), 'DisplayName', names(a) + " vs previous cmd");
    end
    for c = capT, xline(c, 'k--', 'HandleVisibility', 'off'); end
    yline(10, 'r:', '10 deg');
    xlabel('time (s, sensing-time convention)'); ylabel('|motion dir - commanded| (deg)');
    legend('Location', 'best'); title(sprintf('%s: heading tracking error', upper(e.name)));
    exportgraphics(f5, fullfile(figdir, e.name + "_heading_error.png"), 'Resolution', 150);
    close all
end

function step_compute(pur, evs, Ea, T, la, win, act)
    % What env_hardware.step computes per iteration (DECIDE part only).
    pur.heading_velocity(Ea, T, la, win, "closest_next_step");
    for i = 1:numel(evs)
        wi = (norm(T - evs(i).position)^2 - norm(T - pur.position)^2) >= 0;
        evs(i).heading_velocity(pur.position, T, wi, 1, "optimal");
    end
end
