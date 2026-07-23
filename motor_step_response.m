function info = motor_step_response(bot_name, opts)
% MOTOR_STEP_RESPONSE  Measure how long a bot takes to reach a commanded speed.
%
%   info = motor_step_response("evader1")
%   info = motor_step_response("pursuer", vmax=0.15, duration=3, plot=false)
%
% Commands a step to a constant BODY-forward velocity from rest and records the
% mocap position at the mocap frame rate. It then differentiates position into
% speed and reports the actuator response:
%   rise_time    10% -> 90% of the steady-state speed (s)
%   tau          time to reach 63.2% of steady state ~ first-order time constant (s)
%   settle_time  time after which speed stays within +/- settle_band of steady state (s)
%   v_ss         achieved steady-state speed (m/s), vs the commanded vmax
%
% Compare tau/settle_time against your control period dt (0.05 s) to judge how
% much the sample-and-hold assumption is stretched.
%
% Give the bot ~0.5 m of clear space ahead. FUNCTION so onCleanup stops the
% motors on any exit. Requires ROS2 up (uses mocap).

    arguments
        bot_name (1,1) string
        opts.vmax            (1,1) double  = 0.15   % commanded body-forward speed (m/s)
        opts.duration        (1,1) double  = 2.5    % s to hold the step & record
        opts.settle_band     (1,1) double  = 0.10   % +/- fraction of v_ss for "settled"
        opts.smooth_time     (1,1) double  = 0.10   % s window for smoothing the speed
        opts.plot            (1,1) logical = true
        opts.receive_timeout (1,1) double  = 2
        opts.reset           (1,1) logical = false
    end

    %% ---------- connect ----------
    bot  = BotHardware.fromConfig(botConfigs(bot_name));
    node = getMocapNode(opts.reset);
    bot.connect(node, opts.reset);
    cleanup = onCleanup(@() bot.halt()); %#ok<NASGU>

    if ~bot.sense(opts.receive_timeout)
        error("motor_step_response:noPose", ...
              "No mocap pose for '%s'. Is the rigid body tracked?", bot_name);
    end

    fprintf("[%s] step to %.3f m/s (body-forward). Give ~0.5 m clearance. Starting in 2 s...\n", ...
            bot_name, opts.vmax);
    pause(2)

    %% ---------- command the step and record ----------
    t   = zeros(1, 0);
    pos = zeros(2, 0);

    bot.drive_body([opts.vmax; 0]);   % STEP from rest, t = 0 is the command instant
    t0 = tic;
    while toc(t0) < opts.duration
        if bot.sense(opts.receive_timeout)
            t(end+1)     = toc(t0);   %#ok<AGROW>
            pos(:,end+1) = bot.pos;   %#ok<AGROW>
        end
    end
    bot.halt();

    if numel(t) < 10
        error("motor_step_response:tooFewSamples", ...
              "Only %d mocap samples -- check tracking / rate.", numel(t));
    end

    %% ---------- position -> speed ----------
    dt_s  = diff(t);
    seg   = vecnorm(diff(pos, 1, 2));   % distance between consecutive samples
    v_raw = seg ./ dt_s;                % speed at segment midpoints
    t_mid = t(1:end-1) + dt_s/2;

    % Smooth: window of ~smooth_time seconds, in samples.
    n_smooth = max(3, round(opts.smooth_time / median(dt_s)));
    v = movmean(v_raw, n_smooth);

    %% ---------- extract the response ----------
    % Steady state = mean speed over the last 30% of the record.
    ss_mask = t_mid > 0.7*opts.duration;
    v_ss    = mean(v(ss_mask));

    t10 = first_cross(t_mid, v, 0.10*v_ss);
    t63 = first_cross(t_mid, v, 0.632*v_ss);
    t90 = first_cross(t_mid, v, 0.90*v_ss);
    rise_time = t90 - t10;

    outside = abs(v - v_ss) > opts.settle_band*v_ss;
    idx_last = find(outside, 1, 'last');
    if isempty(idx_last)
        settle_time = t_mid(1);
    else
        settle_time = t_mid(min(idx_last+1, numel(t_mid)));
    end

    info = struct('bot', bot_name, 'vmax', opts.vmax, 'v_ss', v_ss, ...
                  'v_fraction', v_ss/opts.vmax, 'rise_time', rise_time, ...
                  'tau', t63, 'settle_time', settle_time, ...
                  't', t_mid, 'v', v);

    %% ---------- report ----------
    fprintf("\n==================================================\n");
    fprintf("Step response: '%s'  (commanded %.3f m/s)\n", bot_name, opts.vmax);
    fprintf("  steady-state speed  = %.3f m/s   (%.0f%% of command)\n", v_ss, 100*v_ss/opts.vmax);
    fprintf("  rise time 10-90%%    = %.3f s\n", rise_time);
    fprintf("  time constant tau   = %.3f s\n", t63);
    fprintf("  settle (+/-%.0f%%)     = %.3f s\n", 100*opts.settle_band, settle_time);
    fprintf("  vs control dt=0.05  : settle is ~%.1f control steps\n", settle_time/0.05);
    fprintf("==================================================\n");

    if opts.plot
        figure; hold on; grid on
        plot(t_mid, v, 'b-', 'LineWidth', 1.5)
        yline(v_ss, 'k--', 'v_{ss}');
        yline(opts.vmax, 'r:', 'commanded');
        if ~isnan(t63), xline(t63, 'g--', '\tau'); end
        if ~isnan(settle_time), xline(settle_time, 'm--', 'settle'); end
        xlabel('time since command (s)'); ylabel('speed (m/s)')
        title(sprintf('%s step response', bot_name))
        hold off
    end
end

function tc = first_cross(t, v, level)
    % First time the (smoothed) speed reaches `level`.
    idx = find(v >= level, 1, 'first');
    if isempty(idx)
        tc = NaN;
    else
        tc = t(idx);
    end
end
