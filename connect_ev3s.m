function ev3s = connect_ev3s(names, opts)
% CONNECT_EV3S  Debug helper: connect the EV3 bricks ONLY (no ROS, no game).
%
% Isolates the multi-brick WiFi connection from env_hardware so we can see
% exactly which brick fails, retry transient failures, and test whether
% connecting all EV3s WITHOUT interleaved ROS subscribers avoids the problem.
%
%   connect_ev3s()                                 % all three, from botConfigs
%   connect_ev3s(["pursuer","evader1"])            % a subset
%   connect_ev3s("evader2", retries=5, pause_between=3)
%
% Uses getEv3, so successful connections are CACHED -- a subsequent
% main_hardware will reuse them instead of reconnecting (a handy workaround if
% the issue turns out to be the reconnect/interleave).

    arguments
        names (1,:) string = ["pursuer","evader1","evader2"]
        opts.pause_between (1,1) double  = 2      % s to wait between bricks
        opts.retries       (1,1) double  = 3      % attempts per brick
        opts.retry_wait    (1,1) double  = 3      % s between attempts
        opts.reset         (1,1) logical = false  % force fresh connections
    end

    ev3s = struct();
    for nm = names
        cfg = botConfigs(nm);
        fprintf("\n=== %s : %s @ %s ===\n", nm, cfg.ev3_serial, cfg.ev3_ip);

        connected = false;
        for attempt = 1:opts.retries
            try
                e = getEv3(cfg.ev3_serial, cfg.ev3_ip, opts.reset);
                ev3s.(char(nm)) = e;
                fprintf("  OK on attempt %d | battery %d%%\n", attempt, e.BatteryLevel);
                connected = true;
                break
            catch ME
                fprintf("  attempt %d FAILED: %s\n", attempt, ME.message);
                if attempt < opts.retries
                    pause(opts.retry_wait);
                end
            end
        end
        if ~connected
            fprintf("  >>> GAVE UP on %s after %d attempts.\n", nm, opts.retries);
        end

        pause(opts.pause_between);   % breathe before the next brick
    end

    % Battery summary for all connected bricks (handy for the matched-battery
    % check -- similar charge keeps the speed ratio alpha ~ 1 as they drain).
    connected_names = string(fieldnames(ev3s))';
    fprintf("\nConnected bricks: %s\n", strjoin(connected_names, ", "));
    if ~isempty(connected_names)
        fprintf("\n--- Battery levels ---\n");
        for nm = connected_names
            fprintf("  %-10s %3d%%\n", nm, ev3s.(char(nm)).BatteryLevel);
        end
    end
end
