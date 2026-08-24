function [flip, info] = check_one_step_flip(pursuer_position, evader_positions, target_position, timestep)
% CHECK_ONE_STEP_FLIP  Test a single 1-pursuer/2-evader/1-target position
% at step k against the PDR -> EDR flip conditions.
%
%   [flip, info] = check_one_step_flip(pursuer_position, evader_positions, target_position, timestep)
%
% Inputs (all in the same units/convention as main.m):
%   pursuer_position  2x1
%   evader_positions  2x2   (columns = evader 1, evader 2)
%   target_position   2x1
%   timestep          scalar, e.g. 0.05
%
% What it checks, at step k -> k+1 (equal speeds, alpha = 1):
%   (i)   [not checked here -- check separately, see below] target should
%         start strictly inside the pursuer dominance region (PDR) against
%         BOTH evaders:
%             ||x_P^k - x_T|| < ||x_1^k - x_T||,  ||x_P^k - x_T|| < ||x_2^k - x_T||
%   (ii)  Pursuer.optimal_pursuer_heading (the "closest_next_step" policy)
%         locks onto one evader -- "chosen" -- based on which evader would
%         be closest to the target after one step of HYPOTHETICAL OPTIMAL
%         evasion (what the pursuer assumes internally, regardless of what
%         the evaders actually do). The other evader is "deviant".
%   (iii) The deviant evader instead heads straight at the target for this
%         one step. That single deviation flips the target into the
%         deviant's dominance region by k+1:
%             ||x_P^{k+1} - x_T|| > ||x_deviant^{k+1} - x_T||
%         even though, had it played optimally instead, the region would
%         NOT have flipped:
%             ||x_deviant^{k+1}_opt - x_T|| >= ||x_P^{k+1} - x_T||
%
% Output:
%   flip_condition  true iff (ii)-(iii) both hold (does NOT check (i) --
%                   do that yourself first, see example below)
%   info            struct with every intermediate quantity so you can
%                   inspect the numbers by hand:
%     .chosen, .deviant           evader indices (1 or 2)
%     .theta_star, .theta_p       per-evader optimal pursuer heading vs.
%                                 the one actually used (matches chosen)
%     .evader_next_opt(:,i)       evader i's next position under optimal play
%     .dist_next_opt(i)           ||evader_next_opt(:,i) - target||
%     .pursuer_next               pursuer's actual next position
%     .evader_next_dev            deviant's next position if it instead
%                                 heads straight at the target
%     .dPT1, .dEdT1_dev            ||pursuer_next - target||, ||evader_next_dev - target||
%     .cond_flip                  (iii) first inequality
%     .cond_optimal_would_not_flip (iii) second inequality
%     .cond_choice_justified      sanity check that "chosen" is indeed the
%                                 more urgent evader under optimal play
%
% Example -- manually testing one candidate IC:
%   pursuer_position = [0; -1];
%   evader_positions = [0, 1.35; 1, 0];
%   target_position  = [0.31225; -0.05];
%   timestep = 0.05;
%
%   dPT0 = norm(pursuer_position - target_position);
%   dET0 = [norm(evader_positions(:,1)-target_position), norm(evader_positions(:,2)-target_position)];
%   strict_pdr0 = all(dPT0 < dET0)          % condition (i) -- check this yourself
%
%   [flip, info] = check_one_step_flip(pursuer_position, evader_positions, target_position, timestep);
%   flip                                     % true/false for (ii)-(iii)
%   info                                     % inspect the numbers
%
% If flip (and strict_pdr0) is true, validate the full trajectory with:
%   env = Environment(2, timestep, 0.2, pursuer_position, evader_positions, target_position);
%   env.set_pursuer_policy("closest_next_step");
%   env.set_evader_policy("straight_to_target");
%   [done, history] = env.simulate(3000, true);   % true = plot it
%   history.termination_reason                    % want "all_evaders_captured"

% pursuer_position = [0;-1];
% evader_positions = [0,0;1.1,-1.05];
% target_position = [0;0];
% timestep = 0.5;

speed = 1;
p = Pursuer(pursuer_position, speed, 0);
n = size(evader_positions, 2);

% Reject initial conditions that are already degenerate at k, before any
% of the flip logic below is meaningful: a capture (pursuer-evader or
% evader-target) that has effectively already happened.
%   evader captures target within one step:  distance <= timestep
%   pursuer captures an evader within one step: distance <= 4*timestep
%   (matches main.m's tolerance=0.2 at its default timestep=0.05)
evader_target_capture_dist  = timestep;
pursuer_evader_capture_dist = 4.1*timestep;

dPE0 = [norm(pursuer_position - evader_positions(:,1)), norm(pursuer_position - evader_positions(:,2))];
dET0 = [norm(evader_positions(:,1) - target_position),   norm(evader_positions(:,2) - target_position)];

already_captured       = any(dPE0 <= pursuer_evader_capture_dist);
already_reached_target = any(dET0 <= evader_target_capture_dist);
valid_ic = ~already_captured && ~already_reached_target;

% Hypothetical optimal next-step positions/distances for every evader --
% this is exactly what Pursuer.optimal_pursuer_heading uses internally
% to decide who to chase, independent of what the evaders actually do.
theta_star = zeros(1,n);
evader_next_opt = zeros(2,n);
dist_next_opt = zeros(1,n);
for i = 1:n
    [psi_i, theta_i] = p.optimal_headings_Ei(evader_positions(:,i), target_position);
    theta_star(i) = theta_i;
    evader_next_opt(:,i) = evader_positions(:,i) + timestep*speed*[cos(psi_i); sin(psi_i)];
    dist_next_opt(i) = norm(evader_next_opt(:,i) - target_position);
end

[~, theta_p] = p.optimal_pursuer_heading(evader_positions, target_position, timestep, true);
[~, chosen] = min(abs(theta_star - theta_p));
other = setdiff(1:n, chosen);
deviant = other(1);

pursuer_next = pursuer_position + timestep*speed*[cos(theta_p); sin(theta_p)];

dir_dev = target_position - evader_positions(:,deviant);
dir_dev = dir_dev / norm(dir_dev);
evader_next_dev = evader_positions(:,deviant) + timestep*speed*dir_dev;

dPT1 = norm(pursuer_next - target_position);
dEdT1_dev = norm(evader_next_dev - target_position);

info = struct();
info.chosen = chosen;
info.deviant = deviant;
info.theta_star = theta_star;
info.theta_p = theta_p;
info.evader_next_opt = evader_next_opt;
info.dist_next_opt = dist_next_opt;
info.pursuer_next = pursuer_next;
info.evader_next_dev = evader_next_dev;
info.dPT1 = dPT1;
info.dEdT1_dev = dEdT1_dev;

% (1) Deviation actually flips the target out of the pursuer's
%     dominance region against the deviant evader (strict, matching
%     Environment.barrier < 0):
%         ||x_P^{k+1} - x_T|| > ||x_deviant^{k+1} - x_T||
cond_flip = dPT1 > dEdT1_dev;

% (2) It is genuinely the deviation causing this -- had the deviant
%     played optimally instead, the region would NOT have flipped:
%         ||x_chosen^{k+1}_opt - x_T|| >= ||x_P^{k+1} - x_T||
cond_optimal_would_not_flip = dist_next_opt(chosen) >= dPT1;

% (3) Consistency check: "chosen" is, by construction, the evader
%     Pursuer.optimal_pursuer_heading judged closest to the target
%     under optimal play -- i.e. the more urgent one -- so
%         ||x_chosen^{k+1}_opt - x_T|| <= ||x_deviant^{k+1}_opt - x_T||
%     always holds once "chosen"/"deviant" are identified correctly;
%     kept here as an assertion, not an extra filter.
cond_choice_justified = dist_next_opt(chosen) <= dist_next_opt(deviant);

info.dPE0 = dPE0;
info.dET0 = dET0;
info.already_captured = already_captured;
info.already_reached_target = already_reached_target;
info.valid_ic = valid_ic;

info.cond_flip = cond_flip;
info.cond_optimal_would_not_flip = cond_optimal_would_not_flip;
info.cond_choice_justified = cond_choice_justified;

% Combined into the actual first output (named cond_flip in this
% function's current signature) so callers checking that single return
% value also pick up the new capture-avoidance filters.
% flip = cond_flip && cond_optimal_would_not_flip && cond_choice_justified && valid_ic;
flip = cond_flip && valid_ic;
end
