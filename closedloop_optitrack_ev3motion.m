function history = closedloop_optitrack_ev3motion(bot_name, goal, opts)
% CLOSEDLOOP_OPTITRACK_EV3MOTION  Drive one bot to a goal using OptiTrack feedback.
%
%   closedloop_optitrack_ev3motion("evader1", [0.5; 0.3])
%   closedloop_optitrack_ev3motion("pursuer", [0; 0], vmax=0.10, tol=0.10)
%
% Per-bot constants come from botConfigs.m; the bot must already be calibrated
% (BotHardware refuses a NaN yaw_offset -- an uncalibrated bot spirals).
%
% This is the single-bot sanity test for the whole hardware stack. It runs the
% SAME path as the game does -- BotHardware.sense -> control law -> drive() ->
% world-to-body rotation -> kiwi Jacobian -> motors -- so if this works, the
% only thing env_hardware adds on top is the strategy and more bots.
%
% It is a FUNCTION on purpose: onCleanup does not fire in a script, so an error
% or a Ctrl-C would leave the motors spinning and the bot would drive off.

    arguments
        bot_name (1,1) string
        goal     (:,1) double
        opts.Kp              (1,1) double = 1.0    % proportional position gain
        opts.vmax            (1,1) double = 0.12   % max commanded speed (m/s)
        opts.tol             (1,1) double = 0.10   % stop radius (m)
        opts.dt              (1,1) double = 0.05   % control period (s)
        opts.timeout         (1,1) double = 60     % abort after this long (s)
        opts.receive_timeout (1,1) double = 2
        opts.max_miss        (1,1) double = 10     % consecutive bad frames before abort
        opts.reset           (1,1) logical = false % force fresh EV3/ROS connections
    end

    goal = goal(:);
    if numel(goal) ~= 2 || ~all(isfinite(goal))
        error("closedloop:badGoal", "goal must be a 2-element finite vector, e.g. [0.5; 0.3].")
    end

    %% ---------- connect ----------
    bot  = BotHardware.fromConfig(botConfigs(bot_name));  % errors if yaw_offset is NaN
    node = getMocapNode(opts.reset);
    bot.connect(node, opts.reset);

    cleanup = onCleanup(@() bot.halt()); %#ok<NASGU>

    if ~bot.sense(opts.receive_timeout)
        error("closedloop:noPose", ...
              "No mocap pose for '%s'. Is the rigid body tracked in Motive?", bot.name);
    end
    p0 = bot.pos;

    fprintf("\n[%s] start (%+.3f, %+.3f)  yaw %+.1f deg\n", ...
            bot.name, p0(1), p0(2), rad2deg(bot.yaw));
    fprintf("[%s] goal  (%+.3f, %+.3f)   %.2f m away\n", ...
            bot.name, goal(1), goal(2), norm(goal - p0));
    fprintf("Starting in 2 s... (Ctrl-C aborts; motors stop safely)\n");
    pause(2)

    %% ---------- control loop ----------
    history.time = [];
    history.pos  = [];
    history.err  = [];

    t0 = tic;
    while true
        if ~bot.sense(opts.receive_timeout)
            bot.halt();                     % never drive on stale/missing data
            if bot.miss >= opts.max_miss
                error("closedloop:mocapLost", ...
                      "[%s] lost the mocap feed for %d frames in a row. Aborting.", ...
                      bot.name, opts.max_miss);
            end
            pause(opts.dt);
            continue
        end

        err  = goal - bot.pos;
        dist = norm(err);

        history.time(end+1) = toc(t0);       %#ok<AGROW>
        history.pos(:,end+1) = bot.pos;      %#ok<AGROW>
        history.err(end+1)  = dist;          %#ok<AGROW>

        fprintf("pos %+.3f %+.3f | yaw %+4.0f deg | err %.3f\n", ...
                bot.pos(1), bot.pos(2), rad2deg(bot.yaw), dist);

        if dist < opts.tol
            fprintf("[%s] goal reached.\n", bot.name);
            break
        end
        if toc(t0) > opts.timeout
            warning("[%s] timed out after %g s (err %.3f).", bot.name, opts.timeout, dist);
            break
        end

        % Proportional control in the WORLD frame, saturated. drive() does the
        % world->body rotation (per-bot yaw_offset) and the kiwi Jacobian.
        vel = opts.Kp * err;
        sp  = norm(vel);
        if sp > opts.vmax
            vel = vel / sp * opts.vmax;
        end
        bot.drive(vel);

        pause(opts.dt)
    end

    bot.halt();
    history.final_err = history.err(end);
    fprintf("[%s] done. final err %.3f m over %.1f s.\n", ...
            bot.name, history.final_err, toc(t0));
end
