% FOLLOWUP  Max per-iteration displacement, capture-sample distances, Exp2 glitch neighbours.
base = '/Users/abinashagasti/IITM/Research/Paper Submissions/Multiplayer Evader/Modified Experiment Images/';
files = {[base 'Experiment 1/Exp1_3.mat'], [base 'Experiment 2/Exp2_1.mat']};
for x = 1:2
    h = load(files{x}).history; t = h.time; N = numel(t); P = h.pursuer_positions; T = h.target_position;
    n = size(h.evader_positions,2); rho = h.tolerance;
    POS = {P}; for i = 1:n, POS{end+1} = squeeze(h.evader_positions(:,i,:)); end %#ok<AGROW>
    capIdx = arrayfun(@(i) find(vecnorm(P - POS{1+i}) < rho, 1), 1:n);
    stopIdx = [N, capIdx]; names = ["P","E1","E2"];
    fprintf('\n===== EXP%d\n', x);
    for a = 1:1+n
        X = POS{a}; d = vecnorm(diff(X,1,2));   % d(j): sample j -> j+1, interval [time(j-1), time(j)]
        js = 2:stopIdx(a)-1;                     % driven intervals (sample 1->2 is before any command)
        [dm, k] = max(d(js)); j = js(k);
        fprintf('%s: max displacement while driven %.4f m (interval %d->%d, [%.3f, %.3f] s, dt %.3f s)\n', ...
            names(a), dm, j, j+1, t(j-1), t(j), t(j)-t(j-1));
        if x == 2 && a == 2
            ok = js(~ismember(js, [12 13]));     % intervals touching glitch sample 13
            [dm2, k2] = max(d(ok)); j2 = ok(k2);
            fprintf('   excluding intervals 12->13, 13->14 (glitch): %.4f m (interval %d->%d, dt %.3f s); 12->14 span %.4f m over %.3f s\n', ...
                dm2, j2, j2+1, t(j2)-t(j2-1), norm(X(:,14)-X(:,12)), t(13)-t(11));
        end
        if a > 1
            fprintf('   post-capture interval %d->%d displacement (coast): %.4f m\n', stopIdx(a), stopIdx(a)+1, ...
                    norm(X(:,min(stopIdx(a)+1,N)) - X(:,stopIdx(a))));
        end
    end
    for i = 1:n
        j = capIdx(i); E = POS{1+i};
        fprintf('E%d capture sample j=%d: |P-E| = %.4f (rho - d = %.4f); sample j-1=%d: |P-E| = %.4f\n', ...
            i, j, norm(P(:,j)-E(:,j)), rho - norm(P(:,j)-E(:,j)), j-1, norm(P(:,j-1)-E(:,j-1)));
    end
    if x == 2
        E1 = POS{2};
        for j = [12 13 14]
            fprintf('Exp2 sample %d (time %.4f, t_sense %.4f): E1 = (%.4f, %.4f), P = (%.4f, %.4f), |P-E1| = %.4f, |E1-T| = %.4f\n', ...
                j, t(j), t(j-1), E1(:,j), P(:,j), norm(P(:,j)-E1(:,j)), norm(E1(:,j)-T));
        end
        fprintf('Paper edit (E1 at 13 := E1 at 14, P at 13): |P-E1| = %.4f, |E1-T| = %.4f\n', ...
            norm(P(:,13)-E1(:,14)), norm(E1(:,14)-T));
    end
end
