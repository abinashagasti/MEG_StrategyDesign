S = load('/Users/abinashagasti/IITM/Research/Paper Submissions/Multiplayer Evader/Modified Experiment Images/Experiment 2/Exp2_1.mat'); h = S.history;
T = h.target_position; P = h.pursuer_positions; E1 = squeeze(h.evader_positions(:,1,:)); E2 = squeeze(h.evader_positions(:,2,:));
fprintf(' j    time    Px      Py     E1x     E1y    E2x     E2y   |PE1|  |PE2|  |E1T|  |E2T|  |PT|  cap\n');
for j = 1:numel(h.time)
  fprintf('%2d %7.4f %6.3f %6.3f %7.3f %7.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %d%d\n', j, h.time(j), P(:,j), E1(:,j), E2(:,j), ...
    norm(P(:,j)-E1(:,j)), norm(P(:,j)-E2(:,j)), norm(E1(:,j)-T), norm(E2(:,j)-T), norm(P(:,j)-T), h.captured_evaders(:,j));
end
S = load('/Users/abinashagasti/IITM/Research/Paper Submissions/Multiplayer Evader/Modified Experiment Images/Experiment 1/Exp1_3.mat'); h = S.history;
T = h.target_position; P = h.pursuer_positions; E1 = squeeze(h.evader_positions(:,1,:)); E2 = squeeze(h.evader_positions(:,2,:));
fprintf('\nEXP1\n');
for j = 1:numel(h.time)
  fprintf('%2d %7.4f %6.3f %6.3f %7.3f %7.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %d%d\n', j, h.time(j), P(:,j), E1(:,j), E2(:,j), ...
    norm(P(:,j)-E1(:,j)), norm(P(:,j)-E2(:,j)), norm(E1(:,j)-T), norm(E2(:,j)-T), norm(P(:,j)-T), h.captured_evaders(:,j));
end
