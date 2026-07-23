classdef BotHardware < handle
    % Hardware interface for ONE EV3 kiwi-drive bot tracked by OptiTrack.
    %
    % Owns everything physical about an agent -- the mocap subscription, the
    % EV3 connection, and the calibration constants -- so that Pursuer/Evader
    % stay pure strategy objects shared with the simulation.
    %
    % Every constant below is PER BOT: the bots differ in wheel radius,
    % geometry, yaw offset and (possibly) motor wiring.
    %
    % Usage:
    %   bot = BotHardware.fromConfig(cfg);   % cfg is a struct, see CONFIG_FIELDS
    %   bot.connect(node);                   % node = getMocapNode()
    %   ok = bot.sense(2);                   % updates bot.pos / bot.yaw
    %   bot.drive([0.1; 0.0]);               % world-frame velocity, m/s
    %   bot.halt();

    properties (Constant)
        % Fields accepted in a config struct (see fromConfig).
        CONFIG_FIELDS = ["name","ev3_ip","ev3_serial","wheel_radius", ...
                         "wheel_offset","yaw_offset","wheel_ports", ...
                         "motor_max_radps","motor_clamp","motor_deadband","up_axis"];
    end

    properties
        % --- identity / connection ---
        name       (1,1) string = ""    % mocap rigid body name; topic = /vrpn_mocap/<name>/pose
        ev3_ip     (1,1) string = ""
        ev3_serial (1,1) string = ""

        % --- calibration (from Track 3) ---
        wheel_radius (1,1) double = 0.025 % r: wheel radius (m)
        wheel_offset (1,1) double = 0.14  % L: centre-to-wheel distance (m)
        yaw_offset   (1,1) double = NaN   % rad, from calibrate_yaw_offset.m.
        % NaN on purpose: an uncalibrated offset silently defaulting to 0 makes
        % the bot spiral, so fromConfig refuses a config that does not set it.

        % --- actuation limits / wiring ---
        wheel_ports     (1,3) string = ["C","A","B"] % wheel k is driven by port wheel_ports(k)
        motor_max_radps (1,1) double = 16            % wheel rad/s at 100% duty
        motor_clamp     (1,1) double = 90            % max |motor percent|
        motor_deadband  (1,1) double = 0             % min |duty| for a NONZERO wheel
        % command (stiction breaker); 0 = off. Raise it for a bot whose wheels
        % slip/stall from rest on a low-grip floor (e.g. tiles).

        % --- frame ---
        up_axis (1,1) char = 'z'   % 'z' -> floor = (x,y)
    end

    properties (SetAccess = private)
        ev3                            % legoev3 object
        motors                         % 1x3 cell of motor objects; motors{k} drives WHEEL k
        sub                            % ros2subscriber on the pose topic
        pos (2,1) double = [NaN; NaN]  % last good floor position (m)
        yaw (1,1) double = NaN         % last good yaw (rad)
        last_stamp (1,1) double = -inf % header stamp of the last good frame (s)
        miss (1,1) double = 0          % consecutive bad/stale frames
        running (1,1) logical = false  % are the motors started?
    end

    methods (Static)

        function bot = fromConfig(cfg)
            % Builds a BotHardware from a config struct. Unknown fields are an
            % error rather than a silent no-op -- a typo'd 'yaw_offet' would
            % otherwise leave the bot uncalibrated.
            if ~isstruct(cfg) || ~isscalar(cfg)
                error("BotHardware:badConfig", "Bot config must be a scalar struct.")
            end
            bot = BotHardware();

            fields = string(fieldnames(cfg));
            unknown = setdiff(fields, BotHardware.CONFIG_FIELDS);
            if ~isempty(unknown)
                error("BotHardware:badConfig", "Unknown bot config field(s): %s.", ...
                      strjoin(unknown, ", "))
            end
            for f = fields'
                bot.(f) = cfg.(f);
            end

            if strlength(bot.name) == 0 || strlength(bot.ev3_ip) == 0 || strlength(bot.ev3_serial) == 0
                error("BotHardware:badConfig", ...
                      "Bot config needs name, ev3_ip and ev3_serial.")
            end
            if ~isfinite(bot.yaw_offset)
                error("BotHardware:notCalibrated", ...
                      "yaw_offset for '%s' is not set. Run calibrate_yaw_offset.m for this bot.", bot.name)
            end
        end

    end

    methods

        function connect(bot, node, reset)
            % Creates the EV3 connection, the motors and the pose subscriber.
            % Both are slow to build, so they are created once and reused; call
            % with reset=true to force a fresh connection (e.g. after a reboot).
            if nargin < 3 || isempty(reset)
                reset = false;
            end
            if reset
                bot.halt();
                bot.ev3 = []; bot.motors = []; bot.sub = [];
                bot.last_stamp = -inf; bot.miss = 0;
            end

            if isempty(bot.ev3)
                % Shared, per-brick connection: opened once and reused so we
                % never hit the EV3's one-connection-at-a-time limit.
                bot.ev3 = getEv3(bot.ev3_serial, bot.ev3_ip, reset);
                bot.motors = cell(1,3);
                for k = 1:3
                    bot.motors{k} = motor(bot.ev3, char(bot.wheel_ports(k)));
                end
                bot.running = false;
            else
                fprintf("[%s] reusing EV3 connection.\n", bot.name);
            end

            if isempty(bot.sub)
                topic = "/vrpn_mocap/" + bot.name + "/pose";
                fprintf("[%s] subscribing to %s ...\n", bot.name, topic);
                bot.sub = ros2subscriber(node, topic, "geometry_msgs/PoseStamped", ...
                                         "Reliability", "besteffort");
            else
                fprintf("[%s] reusing pose subscriber.\n", bot.name);
            end
        end

        function ok = sense(bot, receive_timeout)
            % Reads one pose. Returns false on a dropped OR frozen frame and
            % increments bot.miss; never throws, so the caller decides whether
            % a dropout is fatal (it must stop the motors either way).
            ok = false;

            try
                msg = receive(bot.sub, receive_timeout);
            catch
                bot.miss = bot.miss + 1;
                warning("[%s] missed mocap frame (%d in a row).", bot.name, bot.miss);
                return
            end

            % Frozen feed: when Motive loses a body the last pose keeps being
            % republished, so pos/yaw stay bit-for-bit identical forever and
            % receive() never times out. The header stamp is the only tell.
            stamp = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec)*1e-9;
            if stamp <= bot.last_stamp
                bot.miss = bot.miss + 1;
                warning("[%s] stale/frozen mocap frame (%d in a row).", bot.name, bot.miss);
                return
            end

            [p, y] = readPose(msg, bot.up_axis);
            bot.pos = p(:);   % keep it a 2x1 COLUMN: a row silently implicit-expands
            bot.yaw = y;      % into a 2x2 in the control law, with no error thrown
            bot.last_stamp = stamp;
            bot.miss = 0;
            ok = true;
        end

        function drive(bot, vel_world)
            % Commands a velocity given in the WORLD (mocap) frame, in m/s.
            vel_world = vel_world(:);
            if numel(vel_world) ~= 2 || ~all(isfinite(vel_world))
                error("BotHardware:badVelocity", ...
                      "[%s] drive() needs a finite 2-element velocity.", bot.name)
            end
            if ~isfinite(bot.yaw)
                error("BotHardware:noPose", ...
                      "[%s] drive() called before a valid pose was read.", bot.name)
            end

            % World -> body. Skipping this rotation is the classic failure:
            % the bot spirals or diverges instead of driving to the goal.
            c = cos(bot.yaw + bot.yaw_offset);
            s = sin(bot.yaw + bot.yaw_offset);
            bot.drive_body([c s; -s c] * vel_world);
        end

        function drive_body(bot, vel_body)
            % Commands a velocity in the ROBOT BODY frame ([vx forward; vy left]), m/s.
            %
            % This is the only way to move the bot WITHOUT knowing yaw_offset, so it
            % is what calibrate_yaw_offset uses: it pushes along body-forward and then
            % measures which way the world says the bot actually went. drive() is just
            % this, with the world->body rotation applied first.
            vel_body = vel_body(:);
            if numel(vel_body) ~= 2 || ~all(isfinite(vel_body))
                error("BotHardware:badVelocity", ...
                      "[%s] drive_body() needs a finite 2-element velocity.", bot.name)
            end

            omega = 0;   % holonomic drive: heading is not regulated
            wheel_omega = bot.jacobian() * [omega; vel_body(1); vel_body(2)];

            pct = 100 * wheel_omega / bot.motor_max_radps;
            pct = max(min(pct, bot.motor_clamp), -bot.motor_clamp);
            bot.set_wheel_percent(pct);
        end

        function set_wheel_percent(bot, pct)
            % pct(k) is the duty cycle for WHEEL k, which motors{k} is wired to.
            %
            % Stiction floor: if the whole wheel command is too weak to move the
            % bot from rest, scale ALL wheels up so the LARGEST reaches
            % motor_deadband. Scaling (not per-wheel clamping) preserves the wheel
            % ratios, so the motion DIRECTION is unchanged -- a wheel that should
            % be ~0 stays ~0 (no spurious rotation). Only activates when every
            % wheel is below the floor, i.e. the from-rest stall case. Applied
            % here, the single point every motor write passes through, so both
            % drive() and drive_body() (calibration) get it.
            if bot.motor_deadband > 0
                peak = max(abs(pct));
                if peak > 1e-3 && peak < bot.motor_deadband
                    pct = pct * (bot.motor_deadband / peak);
                end
            end

            if ~bot.running
                for k = 1:3
                    start(bot.motors{k});
                end
                bot.running = true;   % resumes correctly after a dropout halt
            end
            for k = 1:3
                bot.motors{k}.Speed = pct(k);
            end
        end

        function J = jacobian(bot)
            % Kiwi (3-wheel omni) inverse kinematics: wheel_omega = J*[omega; vx; vy].
            r = bot.wheel_radius;
            L = bot.wheel_offset;
            J = (1/r) * [ ...
                -L     1        0
                -L  -0.5  -sqrt(3)/2
                -L  -0.5   sqrt(3)/2 ];
        end

        function halt(bot)
            % Stops all three motors. Safe to call at any time, including from
            % an onCleanup on a half-constructed object -- it must never throw.
            if isempty(bot.motors)
                return
            end
            for k = 1:numel(bot.motors)
                try, bot.motors{k}.Speed = 0; end %#ok<TRYNC>
                try, stop(bot.motors{k});     end %#ok<TRYNC>
            end
            bot.running = false;
        end

        function delete(bot)
            bot.halt();
        end

    end
end
