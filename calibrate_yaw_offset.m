function [yaw_offset, info] = calibrate_yaw_offset(bot_name, opts)
% CALIBRATE_YAW_OFFSET  Measures a bot's yaw offset and its true speed.
%
%   yaw_offset = calibrate_yaw_offset("evader1")
%   [yaw_offset, info] = calibrate_yaw_offset("evader1", push_time=2.0, nreps=5)
%
% YAW_OFFSET is the constant angle between the robot's body "forward" (the +vx
% axis the wheel Jacobian assumes) and the mocap yaw=0 reference. Method: push
% along BODY-forward (drive_body -- no yaw_offset needed), then measure which
% way the WORLD says the bot actually travelled:
%
%     yaw_offset = wrapToPi(world_move_direction - mean_mocap_yaw)
%
% The mean yaw over the push is used, not the starting yaw: these bots rotate a
% little as they translate (mechanical asymmetry), which curves the path, so the
% displacement direction matches the AVERAGE heading. Pushes that spin more than
% max_spin are discarded rather than allowed to bias the average.
%
% It also measures the bot's ACTUAL speed vs the commanded speed. This matters:
% the game assumes all agents move at the SAME speed (alpha = 1), so every bot
% needs a motor_max_radps that makes its real m/s match the command.
%
% Paste BOTH printed values into botConfigs.m.
%
% PREREQ: the rigid body must exist and be tracked in Motive. Re-run this after
% ANY re-wand, ground-plane reset, or rigid-body recreation.

    arguments
        bot_name (1,1) string
        opts.push_speed      (1,1) double = 0.15   % commanded body-forward speed (m/s)
        opts.push_time       (1,1) double = 1.5    % duration of each push (s)
        opts.nreps           (1,1) double = 5      % pushes to average
        opts.min_disp        (1,1) double = 0.05   % discard a push that moves less (m)
        opts.max_spin        (1,1) double = 90     % deg; sanity cap only (mean-yaw
                                                   % already compensates normal spin)
        opts.receive_timeout (1,1) double = 2
        opts.reset           (1,1) logical = false % force fresh EV3/ROS connections
    end

    %% ---------- connect ----------
    cfg = botConfigs(bot_name);
    % The offset is what we are measuring, so it is not known yet. Pushes go
    % through drive_body(), which never uses it -- this placeholder only gets
    % past BotHardware's "refuse to run uncalibrated" check.
    cfg.yaw_offset = 0;

    bot  = BotHardware.fromConfig(cfg);
    node = getMocapNode(opts.reset);
    bot.connect(node, opts.reset);

    % A function (not a script) so onCleanup fires on error and on Ctrl-C.
    cleanup = onCleanup(@() bot.halt()); %#ok<NASGU>

    fprintf("\nCalibrating '%s'. Give it ~1 m of clear space AHEAD. Starting in 3 s...\n", bot.name);
    pause(3)

    %% ---------- pushes ----------
    offsets = nan(1, opts.nreps);   % NaN = discarded (spun too much)
    speeds  = nan(1, opts.nreps);

    for k = 1:opts.nreps
        fprintf("\n--- Push %d/%d ---\n", k, opts.nreps);

        if ~bot.sense(opts.receive_timeout)
            error("calibrate_yaw_offset:noPose", ...
                  "No mocap pose for '%s'. Is the rigid body tracked in Motive?", bot.name);
        end
        p0 = bot.pos;  yaw0 = bot.yaw;

        bot.drive_body([opts.push_speed; 0]);   % PURE body-forward

        % Sample throughout the push so we can use the MEAN heading.
        yaws  = yaw0;
        p_end = p0;
        t0    = tic;
        while toc(t0) < opts.push_time
            if bot.sense(opts.receive_timeout)
                p_end = bot.pos;
                yaws(end+1) = bot.yaw; %#ok<AGROW>
            end
            pause(0.05);
        end
        elapsed = toc(t0);
        bot.halt();

        pause(0.5);                              % let it coast to a stop
        p_final = p_end;
        if bot.sense(opts.receive_timeout)
            p_final = bot.pos;
            yaws(end+1) = bot.yaw;
        end
        yaw1 = yaws(end);

        % Direction from the FULL displacement; speed from the powered phase only
        % (p_final includes the coast, which would inflate the speed estimate).
        d          = p_final - p0;
        disp_mag   = norm(d);
        move_dir   = atan2(d(2), d(1));
        mean_yaw   = atan2(mean(sin(yaws)), mean(cos(yaws)));   % circular mean
        net_spin   = wrapToPi(yaw1 - yaw0);

        offsets(k) = wrapToPi(move_dir - mean_yaw);
        speeds(k)  = norm(p_end - p0) / elapsed;

        fprintf("  moved %.3f m | speed %.3f m/s (commanded %.3f)\n", ...
                disp_mag, speeds(k), opts.push_speed);
        fprintf("  world dir %+.1f | mean yaw %+.1f | spin %+.1f | offset %+.1f  (deg)\n", ...
                rad2deg(move_dir), rad2deg(mean_yaw), rad2deg(net_spin), rad2deg(offsets(k)));

        % Tiny displacement -> the direction is noise AND the speed is a stall;
        % discard the rep from BOTH. A big spin with real displacement is fine:
        % mean-yaw already compensates it, so keep the speed and (up to a sanity
        % cap) the offset.
        if disp_mag < opts.min_disp
            warning("  Tiny displacement (%.3f m) -> discarding rep. Check wiring, battery, or the frame.", disp_mag);
            offsets(k) = NaN;
            speeds(k)  = NaN;
        elseif abs(net_spin) > deg2rad(opts.max_spin)
            warning("  Spun %.1f deg (> %g) -> keeping speed, discarding from the yaw average.", ...
                    rad2deg(net_spin), opts.max_spin);
            offsets(k) = NaN;
        end
    end

    bot.halt();

    %% ---------- result ----------
    valid = offsets(~isnan(offsets));
    if isempty(valid)
        error("calibrate_yaw_offset:allSpun", ...
              ["Every push spun more than %g deg -- no trustworthy reading for '%s'. " ...
               "Reduce push_time or push_speed."], opts.max_spin, bot.name);
    end

    yaw_offset = atan2(mean(sin(valid)), mean(cos(valid)));   % circular mean
    spread     = rad2deg(max(valid) - min(valid));

    v_actual  = mean(speeds, 'omitnan');
    frac      = v_actual / opts.push_speed;
    suggested = cfg.motor_max_radps * frac;   % lower radps => higher duty => faster

    info = struct('bot', bot.name, 'offsets_deg', rad2deg(offsets), 'speeds', speeds, ...
                  'spread_deg', spread, 'mean_speed', v_actual, ...
                  'speed_fraction', frac, 'suggested_motor_max_radps', suggested);

    fprintf("\n==================================================\n");
    fprintf("Bot '%s'\n", bot.name);
    fprintf("  yaw_offset  = %+.4f rad (%+.1f deg)\n", yaw_offset, rad2deg(yaw_offset));
    if spread > 20
        fprintf("  spread      = %.1f deg over %d/%d pushes   <-- HIGH, recheck\n", ...
                spread, numel(valid), opts.nreps);
    else
        fprintf("  spread      = %.1f deg over %d/%d pushes   (consistent)\n", ...
                spread, numel(valid), opts.nreps);
    end
    fprintf("  true speed  = %.3f m/s at a commanded %.3f m/s  (%.0f%%)\n", ...
            v_actual, opts.push_speed, 100*frac);
    fprintf("  motor_max_radps -> %.1f   (currently %.1f)\n", suggested, cfg.motor_max_radps);
    fprintf("\n  Paste into botConfigs.m under '%s':\n", bot.name);
    fprintf("      'yaw_offset',      %.4f, ...\n", yaw_offset);
    fprintf("      'motor_max_radps', %.1f);\n", suggested);
    fprintf("==================================================\n");
end
