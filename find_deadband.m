function find_deadband(bot_name, opts)
% FIND_DEADBAND  Find a bot's from-rest stiction duty (its motor_deadband).
%
%   find_deadband("evader1")
%   find_deadband("evader1", p_start=10, p_end=70, p_step=2, hold=1.5)
%
% Ramps the PEAK wheel duty for a body-forward push and drives the bot in short
% bursts, coming to REST between each so you measure STATIC friction (not
% kinetic). Watch the printout and note the duty % where the bot FIRST breaks
% from a standstill on the current floor. Set
%
%     'motor_deadband', <that duty + ~3>
%
% in botConfigs.m for this bot. Leave it 0 / omit for bots that never stall.
%
% EV3-only: does not use OptiTrack, so ROS need not be running. It is a FUNCTION
% so onCleanup stops the motors on any exit (error / Ctrl-C / normal finish).

    arguments
        bot_name (1,1) string
        opts.p_start (1,1) double = 5      % first duty % to try
        opts.p_end   (1,1) double = 70     % last duty % to try
        opts.p_step  (1,1) double = 2      % duty increment
        opts.hold    (1,1) double = 1.5    % s driving at each level
        opts.rest    (1,1) double = 1.0    % s at rest between levels (for STATIC friction)
        opts.reset   (1,1) logical = false % force a fresh EV3 connection
    end

    cfg = botConfigs(bot_name);

    % wheel_ports is not stored in botConfigs; fall back to the BotHardware default.
    ports = ["C","A","B"];   % wheel k is driven by ports(k)
    if isfield(cfg, 'wheel_ports') && ~isempty(cfg.wheel_ports)
        ports = string(cfg.wheel_ports);
    end

    e = getEv3(cfg.ev3_serial, cfg.ev3_ip, opts.reset);
    m = cell(1,3);
    for k = 1:3
        m{k} = motor(e, char(ports(k)));
    end

    cleanup = onCleanup(@() stop_all(m)); %#ok<NASGU>

    % Body-forward wheel pattern (from J*[0; vx; 0]), normalised so the largest
    % wheel equals the commanded peak duty p.
    pattern = [1; -0.5; -0.5];
    pattern = pattern / max(abs(pattern));

    fprintf("\nRamping '%s' body-forward. Note the duty where it FIRST moves from rest.\n\n", bot_name);
    for p = opts.p_start:opts.p_step:opts.p_end
        pct = p * pattern;
        for k = 1:3
            m{k}.Speed = pct(k);
            start(m{k});
        end
        fprintf("peak duty %2d%% -- moving?\n", p);
        pause(opts.hold);
        stop_all(m);
        pause(opts.rest);      % settle to REST so the next try measures static friction
    end

    stop_all(m);
    fprintf("\nDone. Set  'motor_deadband', <first-moving duty + ~3>  in botConfigs.m for '%s'.\n", bot_name);
end

function stop_all(m)
    for k = 1:numel(m)
        try, m{k}.Speed = 0; end %#ok<TRYNC>
        try, stop(m{k});     end %#ok<TRYNC>
    end
end
