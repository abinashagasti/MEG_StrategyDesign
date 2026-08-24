function c = flip_feasibility_constraints(v, pursuer_position, target_position, timestep)
    x1 = v(1:2); x2 = v(3:4);
    dPT0  = norm(pursuer_position - target_position);
    dE1T0 = norm(x1 - target_position);
    dE2T0 = norm(x2 - target_position);

    [~, info] = check_one_step_flip(pursuer_position, [x1, x2], target_position, timestep);

    c = [ dPT0 - dE1T0;
          dPT0 - dE2T0;
          info.dEdT1_dev - info.dPT1;
          %info.dPT1 - info.dist_next_opt(info.chosen);
          4.1*timestep - info.dPE0(1);
          4.1*timestep - info.dPE0(2);
          timestep   - info.dET0(1);
          timestep   - info.dET0(2) ];
end