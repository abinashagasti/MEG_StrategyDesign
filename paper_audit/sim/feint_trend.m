% FEINT_TREND  Per-5s-window min evader-target and pursuer-target distance in the feint runs.
for f = ["feint_example4_dt0025.mat", "feint_example4_dt0050.mat"]
    h = load(f).hF; T = h.target_position; t = h.time;
    dE = squeeze(min(vecnorm(h.evader_positions - T), [], 2))';   % min over evaders
    dPT = vecnorm(h.pursuer_positions - T);
    fprintf('\n%s\n window   min|E-T|   max|P-T|   mean min|P-E|\n', f);
    for w = 0:5:35
        m = t >= w & t < w + 5;
        dPE = min(squeeze(vecnorm(reshape(h.pursuer_positions,2,1,[]) - h.evader_positions)), [], 1);
        fprintf(' %2d-%2d s   %.4f     %.4f     %.4f\n', w, w+5, min(dE(m)), max(dPT(m)), mean(dPE(m)));
    end
end
