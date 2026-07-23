classdef env_hardware < Environment
    % Hardware execution layer for the MEG pursuit-evasion game.
    %
    % Same game, same strategy: step() overrides Environment.step() and swaps
    % the simulated integration (pos <- pos + dt*velocity) for
    %
    %   SENSE      read every agent's pose from OptiTrack (BotHardware.sense)
    %   DECIDE     UNCHANGED strategy code -- Pursuer.heading_velocity and
    %              Environment.return_evader_velocities, exactly as in sim
    %   TRANSFORM  world velocity -> body frame -> wheel speeds (BotHardware.drive)
    %   ACTUATE    write motor speeds on each EV3
    %
    % Nothing in this class knows about yaw offsets, Jacobians or motor ports:
    % those are per-bot and live inside BotHardware. Nothing in this class
    % reimplements the strategy either -- it only moves 2x1 world-frame vectors
    % between the strategy objects and the bots.
    %
    % Usage (see main_hardware.m):
    %   env = env_hardware(pursuer_cfg, evader_cfgs, target_position);
    %   history = env.run();
    %
    % ALWAYS drive the bots through run() (a method) and never from a script:
    % onCleanup does not fire in a script, so an error or Ctrl-C would leave
    % the motors spinning and the bots would drive off.

    properties
        pursuer_bot                  % BotHardware for the pursuer
        evader_bots                  % 1xn BotHardware, aligned with env.evaders
        node                         % shared ros2node

        max_speed (1,1) double       % physical agent speed (m/s), see step()
        receive_timeout (1,1) double % s to wait for each pose message
        max_miss (1,1) double        % consecutive bad frames before aborting
        arena_limits double          % [xmin xmax ymin ymax] (m), [] disables
    end

    methods

        function env = env_hardware(pursuer_cfg, evader_cfgs, target_position, opts)
            arguments
                pursuer_cfg (1,1) struct
                evader_cfgs
                target_position (:,1) double
                opts.timestep (1,1) double = 0.05      % control period (s)
                opts.tolerance (1,1) double = 0.30     % capture radius (m); the bots
                                                       % are large, and this doubles as
                                                       % collision avoidance
                opts.policy (1,1) string = "closest_next_step"
                opts.max_speed (1,1) double = 0.12     % m/s
                opts.receive_timeout (1,1) double = 2
                opts.max_miss (1,1) double = 10
                opts.arena_limits double = []
                opts.reset (1,1) logical = false       % force fresh EV3/ROS connections
            end

            evader_cfgs = env_hardware.normalize_cfgs(evader_cfgs);
            n = numel(evader_cfgs);

            % Bring up ROS2 and the bots BEFORE the superclass call, so the game
            % can be initialised from the bots' REAL positions rather than from
            % hand-typed ones. (MATLAB forbids touching env before the superclass
            % constructor runs, so these stay local for now.)
            node = getMocapNode(opts.reset);

            pursuer_bot = BotHardware.fromConfig(pursuer_cfg);
            pursuer_bot.connect(node, opts.reset);

            evader_bots = BotHardware.empty(1,0);
            for i = 1:n
                evader_bots(i) = BotHardware.fromConfig(evader_cfgs{i});
                evader_bots(i).connect(node, opts.reset);
            end

            if ~pursuer_bot.sense(opts.receive_timeout)
                error("env_hardware:noPose", ...
                      "No mocap pose for pursuer rigid body '%s'. Is it tracked in Motive?", ...
                      pursuer_bot.name);
            end
            evader_positions = zeros(2, n);
            for i = 1:n
                if ~evader_bots(i).sense(opts.receive_timeout)
                    error("env_hardware:noPose", ...
                          "No mocap pose for evader rigid body '%s'. Is it tracked in Motive?", ...
                          evader_bots(i).name);
                end
                evader_positions(:,i) = evader_bots(i).pos;
            end

            env = env@Environment(n, opts.timestep, opts.tolerance, ...
                                  pursuer_bot.pos, evader_positions, ...
                                  target_position(:), opts.policy);

            env.node            = node;
            env.pursuer_bot     = pursuer_bot;
            env.evader_bots     = evader_bots;
            env.max_speed       = opts.max_speed;
            env.receive_timeout = opts.receive_timeout;
            env.max_miss        = opts.max_miss;
            env.arena_limits    = opts.arena_limits;

            env.report_state("Initial state");
        end

        % ---------------------------------------------------------------- run

        function history = run(env, max_steps, max_time)
            % Plays the game on the real bots until capture, an evader reaching
            % the target, max_steps, or max_time.
            arguments
                env
                max_steps (1,1) double = 2000
                max_time  (1,1) double = 120   % s, hard wall-clock stop
            end

            % THE reason this is a method and not a script: onCleanup only fires
            % when a function workspace is destroyed. Here it fires on a normal
            % return, on an error, and on Ctrl-C -- so the motors always stop.
            cleanup = onCleanup(@() env.halt_all()); %#ok<NASGU>

            history.time             = zeros(1, max_steps+1);
            history.pursuer_positions = nan(2, max_steps+1);
            history.evader_positions  = nan(2, env.evader_numbers, max_steps+1);
            history.captured_evaders  = false(env.evader_numbers, max_steps+1);
            history.target_position   = env.target_position;
            [history.pursuer_positions(:,1), history.evader_positions(:,:,1)] = env.snapshot();

            fprintf("\nRunning. Ctrl-C aborts safely (motors stop).\n\n");
            t0 = tic;
            done = false;
            k = 0;

            while ~done && k < max_steps
                if toc(t0) > max_time
                    warning("env_hardware:timeout", "Stopped after %.0f s without termination.", max_time);
                    break
                end

                done = env.step();
                k = k + 1;

                history.time(k+1) = toc(t0);
                [history.pursuer_positions(:,k+1), history.evader_positions(:,:,k+1)] = env.snapshot();
                history.captured_evaders(:,k+1) = env.captured_evaders';
            end

            env.halt_all();

            final = k + 1;
            history.time              = history.time(1:final);
            history.pursuer_positions = history.pursuer_positions(:,1:final);
            history.evader_positions  = history.evader_positions(:,:,1:final);
            history.captured_evaders  = history.captured_evaders(:,1:final);
            history.step_count        = k;
            history.terminated        = done;

            if done && all(env.captured_evaders)
                history.termination_reason = "all_evaders_captured";
            elseif done
                history.termination_reason = "evader_reached_target";
            elseif toc(t0) > max_time
                history.termination_reason = "timeout";
            else
                history.termination_reason = "max_steps_reached";
            end

            fprintf("\n");
            env.report_state("Final state");
            fprintf("Result: %s after %d steps (%.1f s).\n", ...
                    history.termination_reason, k, history.time(end));
        end

        % --------------------------------------------------------------- step

        function done = step(env)
            % One sample-and-hold step on real hardware. Mirrors
            % Environment.step(), with sensing in place of the initial state and
            % actuation in place of the Euler integration.
            done = false;

            % ---------- 1. SENSE ----------
            if ~env.sense_all()
                env.halt_all();                     % never drive on stale data
                if env.worst_miss() >= env.max_miss
                    error("env_hardware:mocapLost", ...
                          "Lost the mocap stream for %d consecutive frames (body occluded?). Aborting.", ...
                          env.max_miss);
                end
                pause(env.timestep);
                return                              % retry on the next step
            end
            env.check_arena();

            % ---------- 2. TERMINATION ----------
            env.updateTermination();
            for i = find(env.captured_evaders)
                env.evader_bots(i).halt();          % a captured evader is out of the game
            end
            if all(env.captured_evaders)
                fprintf("All evaders captured.\n");
                env.halt_all();
                done = true;
                return
            end

            active = find(~env.captured_evaders);
            for i = active
                if norm(env.evaders(i).position - env.target_position) < env.capture_tolerance && ...
                   norm(env.pursuer.position - env.target_position) > 2*env.capture_tolerance
                    fprintf("%s reached the target.\n", env.evaders(i).name);
                    env.halt_all();
                    done = true;
                    return
                end
            end

            % ---------- 3. DECIDE (shared, unchanged strategy) ----------
            win = env.check_initialization(env.evaders(active), false);

            % The strategy is written for UNIT speed: heading_velocity returns a
            % vector of magnitude 1, and its one-step lookahead advances each
            % evader by `timestep` metres. A real bot covers max_speed*timestep
            % metres per control period, so pass THAT as the lookahead distance
            % and scale the returned unit velocity into m/s below. Pursuer and
            % evaders share one max_speed, which preserves the speed ratio
            % alpha = 1 that the game assumes.
            lookahead = env.max_speed * env.timestep;

            pursuer_velocity = env.pursuer.heading_velocity( ...
                env.return_evader_positions(env.evaders(active)), ...
                env.target_position, lookahead, win, env.pursuer_policy);
            evader_velocities = env.return_evader_velocities();

            % ---------- 4/5. TRANSFORM + ACTUATE ----------
            % drive() does the world->body rotation (per-bot yaw offset) and the
            % kiwi Jacobian; it is the only place that touches a motor.
            env.pursuer_bot.drive(env.max_speed * pursuer_velocity);
            for i = active
                env.evader_bots(i).drive(env.max_speed * evader_velocities(:,i));
            end

            pause(env.timestep)
        end

        % ------------------------------------------------------------- helpers

        function ok = sense_all(env)
            % Refreshes every agent's position from mocap. Returns false if any
            % agent still IN the game produced a bad frame; a captured evader is
            % tracked for the log only, and its dropouts must not abort the run.
            ok = env.pursuer_bot.sense(env.receive_timeout);
            if ok
                env.pursuer.updatePos(env.pursuer_bot.pos);
            end

            for i = 1:env.evader_numbers
                good = env.evader_bots(i).sense(env.receive_timeout);
                if good
                    env.evaders(i).updatePos(env.evader_bots(i).pos);
                elseif ~env.captured_evaders(i)
                    ok = false;
                end
            end
        end

        function m = worst_miss(env)
            % Longest dropout streak among the agents still in the game.
            m = env.pursuer_bot.miss;
            for i = find(~env.captured_evaders)
                m = max(m, env.evader_bots(i).miss);
            end
        end

        function check_arena(env)
            % Optional geo-fence: abort (and, via onCleanup, stop the motors) if
            % an agent leaves the tracked area.
            if isempty(env.arena_limits)
                return
            end
            lim = env.arena_limits;
            bots = [env.pursuer_bot, env.evader_bots];
            for b = bots
                p = b.pos;
                if p(1) < lim(1) || p(1) > lim(2) || p(2) < lim(3) || p(2) > lim(4)
                    error("env_hardware:outOfBounds", ...
                          "'%s' left the arena at (%.2f, %.2f). Aborting.", b.name, p(1), p(2));
                end
            end
        end

        function halt_all(env)
            % Stops every motor on every bot. Must never throw: it is the
            % onCleanup target, and may run on a partially built object.
            try, env.pursuer_bot.halt(); end %#ok<TRYNC>
            for i = 1:numel(env.evader_bots)
                try, env.evader_bots(i).halt(); end %#ok<TRYNC>
            end
        end

        function [pursuer_position, evader_positions] = snapshot(env)
            pursuer_position = env.pursuer.position;
            evader_positions = env.return_evader_positions(env.evaders);
        end

        function report_state(env, header)
            if nargin < 2
                header = "State";
            end
            fprintf("--- %s ---\n", header);
            fprintf("  target                 (%+.2f, %+.2f)\n", ...
                    env.target_position(1), env.target_position(2));
            fprintf("  pursuer  %-10s  (%+.2f, %+.2f)  yaw %+6.1f deg\n", ...
                    env.pursuer_bot.name, env.pursuer.position(1), env.pursuer.position(2), ...
                    rad2deg(env.pursuer_bot.yaw));
            for i = 1:env.evader_numbers
                status = "";
                if env.captured_evaders(i)
                    status = "  [captured]";
                end
                fprintf("  evader%-2d %-10s  (%+.2f, %+.2f)  yaw %+6.1f deg%s\n", ...
                        i, env.evader_bots(i).name, ...
                        env.evaders(i).position(1), env.evaders(i).position(2), ...
                        rad2deg(env.evader_bots(i).yaw), status);
            end
            % Positive barrier => the target is in the pursuer's dominance
            % region, i.e. the pursuer can win from here.
            b = env.barrier(env.evaders);
            if b >= 0
                fprintf("  barrier %+.3f  -> pursuer wins from this initialization\n", b);
            else
                fprintf("  barrier %+.3f  -> EVADERS win from this initialization\n", b);
            end
        end

        function plot_history(env, history) %#ok<INUSL>
            % Quick trajectory plot in the mocap frame.
            figure; hold on; grid on; axis equal
            xlabel('x (m)'); ylabel('y (m)'); title('MEG hardware run')
            plot(history.pursuer_positions(1,:), history.pursuer_positions(2,:), 'r-', 'LineWidth', 1.5)
            plot(history.pursuer_positions(1,end), history.pursuer_positions(2,end), 'r.', 'MarkerSize', 30)
            for i = 1:size(history.evader_positions, 2)
                ex = squeeze(history.evader_positions(1,i,:));
                ey = squeeze(history.evader_positions(2,i,:));
                plot(ex, ey, 'b-', 'LineWidth', 1.5)
                plot(ex(end), ey(end), 'b.', 'MarkerSize', 30)
            end
            plot(history.target_position(1), history.target_position(2), 'g.', 'MarkerSize', 30)
            hold off
        end

    end

    methods (Static, Access = private)

        function cfgs = normalize_cfgs(cfgs)
            % Accepts a struct array or a cell array of structs; returns a cell
            % array. (A struct array forces every bot to declare the same
            % fields, which is a nuisance when the bots differ.)
            if iscell(cfgs)
                return
            end
            if isstruct(cfgs)
                cfgs = num2cell(cfgs);
                return
            end
            error("env_hardware:badConfig", ...
                  "evader_cfgs must be a struct array or a cell array of structs.");
        end

    end
end
