classdef Environment < handle
    properties
        motion_space_dimension % dimension of motion space
        evader_numbers % number of evaders in the game
        pursuer_speed % pursuer speed
        evader_speeds % array of evader speeds
        target_position % position of the target
        timestep % timestep value
        captured_evaders % boolean array containing capture status of evaders
        pursuer % pursuer object
        evaders % evader object array
        capture_tolerance % tolerance for point capture
        alpha % speed ratio array
        pursuer_policy % pursuer heading policy used in step and simulate
        convex_optimization_flag % if the optimization in non-heuristic 
        % methods is over the subset of the pareto optimal set where all
        % di functions are concave
    end

    methods

        function env = Environment(n, timestep, varargin)
            if numel(varargin) < 3
                error("Wrong number of inputs for Environment.")
            end

            if isnumeric(varargin{1}) && isscalar(varargin{1})
                if numel(varargin) < 4
                    error("Environment inputs with tolerance must be: tolerance, pursuer_position, evader_positions, target_position.")
                end
                tolerance = varargin{1};
                pursuer_position = varargin{2};
                evader_positions = varargin{3};
                target_position = varargin{4};
                if numel(varargin) >= 5
                    pursuer_policy = varargin{5};
                else
                    pursuer_policy = "closest_next_step";
                end
            else
                tolerance = 0.01;
                pursuer_position = varargin{1};
                evader_positions = varargin{2};
                target_position = varargin{3};
                if numel(varargin) >= 4
                    pursuer_policy = varargin{4};
                else
                    pursuer_policy = "closest_next_step";
                end
            end

            env.evader_numbers = n;
            env.motion_space_dimension = 2;
            env.pursuer_speed = 1;
            env.evader_speeds = ones(1,n);
            env.timestep = timestep;
            env.captured_evaders = boolean(zeros(1,env.evader_numbers));
            env.evaders = Evader.empty(env.evader_numbers,0);
            env.capture_tolerance = tolerance; 
            env.alpha = env.evader_speeds/env.pursuer_speed;
            env.convex_optimization_flag = 0;
            env.pursuer_policy = string(pursuer_policy);
            % This is not required and must be removed in future iteration.

            % Initialize pursuer object instances 
            if ~isempty(pursuer_position)
                env.pursuer = Pursuer(pursuer_position, env.pursuer_speed, env.convex_optimization_flag);
            else
                env.pursuer = Pursuer(rand(env.motion_space_dimension,1), env.pursuer_speed, env.convex_optimization_flag);
            end
            % Remove the convex_optimization_flag from pursuer objects
            % initializations
            
            % Initialize evader object instances
            if ~isempty(evader_positions)
                for i=1:env.evader_numbers
                   env.evaders(i) = Evader(evader_positions(:,i), env.evader_speeds(i), i);
                end
            else
                for i=1:env.evader_numbers
                   env.evaders(i) = Evader(rand(env.motion_space_dimension,1), env.evader_speeds(i), i);
                end
            end
            if ~isempty(target_position)
                env.target_position = target_position;
            else
                env.target_position = rand(env.motion_space_dimension,1);
            end
        end

        function update_target(env, target_position)
            % Updates the target position
            sz_input = size(target_position);
            sz_output = size(env.target_position);
            if sz_input~=sz_output
                if sz_input==fliplr(sz_output)
                    env.target_position = target_position';
                else
                    error('Wrong position dimensions.')
                end
            else
                env.target_position = target_position;
            end
        end

        function set_pursuer_policy(env, pursuer_policy)
            env.pursuer_policy = string(pursuer_policy);
        end
        
        function reset(env)
            % Resets the pursuer and evaders to random initializations
            env.pursuer.updatePos(rand(env.motion_space_dimension,1));
            for i=1:env.evader_numbers
                env.evaders(i).updatePos(Evader(rand(env.motion_space_dimension,1), env.evader_speeds(i)));
            end
        end

        function update_Environment(env,pursuer_position, evader_positions, target_position)
            % Updates pursuer, evader, and target positions
            env.pursuer.updatePos(pursuer_position)
            for i=1:env.evader_numbers
                env.evaders(i).updatePos(evader_positions(:,i));
            end
            env.target_position = target_position;
        end

        function barrier_value = barrier(env, evaders)
            % Gets barrier value of the set of input evaders.The barrier 
            % value is positive iff the target is in pursuer dominance region.
            barriers = zeros(length(evaders),1);
            % disp(length(evaders))
            for i=1:length(evaders)
                barriers(i) = norm(env.target_position - evaders(i).position)^2 - env.alpha(i)^2*norm(env.target_position - env.pursuer.position)^2;
            end
            barrier_value = min(barriers);
        end

        function win = check_initialization(env,evaders,display_info)
            % Given a set of evaders, returns true if the target lies in
            % the pursuer dominance region. 
            if env.barrier(evaders)<0
                win = false;
                if display_info
                    disp("The current initialization results in evaders winning. Please reset the environment and try again.")
                end
            else
                win = true;
                if display_info
                    disp("The current initialization results in pursuers winning. You may proceed.")
                end
            end
        end

        function plot_current_positions(env)
            % Plots the current positions of the agents
            hold on
            for i=1:env.evader_numbers
                plot(env.evaders(i).position(1), env.evaders(i).position(2), '.', 'color', 'b', 'MarkerSize', 30)
            end
            plot(env.target_position(1), env.target_position(2), '.', 'color', 'g', 'MarkerSize', 30)
            plot(env.pursuer.position(1), env.pursuer.position(2), '.', 'color', 'r', 'MarkerSize', 30)
            hold off
        end

        function evader_names = return_evader_names(env)
            % Returns a n-sized vector array of the evader name
            evader_names = strings(1,env.evader_numbers);
            for i=1:env.evader_numbers
                env.evaders(i).name = "evader"+int2str(env.evaders(i).index);
                evader_names(i) = env.evaders(i).name;
            end
        end

        function evader_positions = return_evader_positions(env,evaders)
            % Returns a 2*n array of evader positions collected from all
            % the evader objects. 
            evader_positions = zeros(env.motion_space_dimension,length(evaders));
            for i=1:length(evaders)
                evader_positions(:,i) = evaders(i).position;
            end
        end

        function evader_velocities = return_evader_velocities(env)
            % Returns a 2*n array of evader velocities
            evader_velocities = zeros(env.motion_space_dimension,env.evader_numbers);
            for i=1:env.evader_numbers
                win = env.check_initialization(env.evaders(i),false);
                % win = true;
                evader_velocities(:,i) = env.evaders(i).heading_velocity(env.pursuer.position, env.target_position,win,env.alpha(i));
            end
        end

        function updateTermination(env)
            % Updates termination status of each evader
            for i=1:env.evader_numbers
                if norm(env.pursuer.position-env.evaders(i).position)<env.capture_tolerance
                    env.captured_evaders(i) = true;
                end
            end
        end

        function done = step(env)
            % This is a step function which takes a single step in the
            % sample and hold formulation. 

            env.updateTermination(); % Updates capture status of each evader.
            % if any(env.captured_evaders)
            %     done = true;
            %     return
            % end
            done = false;
            % Return true if all evaders are captured. 
            if all(env.captured_evaders)
                done = true;
                return
            end
            evader_list = 1:env.evader_numbers;
            % If there are uncaptured evaders, check if any of them have
            % reached the target. 
            if ~done
                for i=evader_list(~env.captured_evaders)
                    if norm(env.evaders(i).position- env.target_position)<env.capture_tolerance && norm(env.pursuer.position-env.target_position)>2*env.capture_tolerance
                        done = true;
                        disp_message = strcat(env.evaders(i).name,' reached the target.');
                        disp(disp_message)
                        return
                    end
                end
            end
            % Check win condition to pass as input to determine agent
            % velocities
            win = env.check_initialization(env.evaders(evader_list(~env.captured_evaders)),false);
            pursuer_velocity = env.pursuer.heading_velocity(env.return_evader_positions(env.evaders(evader_list(~env.captured_evaders))),env.target_position,env.timestep,win,env.pursuer_policy);
            evader_velocities = env.return_evader_velocities();
            % Update agent positions after obtaining their velocities.
            env.pursuer.updatePos(env.pursuer.position + env.timestep*pursuer_velocity);
            for i=evader_list(~env.captured_evaders)
                env.evaders(i).updatePos(env.evaders(i).position + env.timestep*evader_velocities(:,i));
            end
        end

        function [done, history] = simulate(env, max_steps, plot_flag)
            % Simulates the game until termination or until max_steps.
            if nargin < 2 || isempty(max_steps)
                max_steps = 10000;
            end
            if nargin < 3 || isempty(plot_flag)
                plot_flag = false;
            end

            pursuer_positions = zeros(env.motion_space_dimension, max_steps + 1);
            evader_positions = zeros(env.motion_space_dimension, env.evader_numbers, max_steps + 1);
            captured_evader_history = false(env.evader_numbers, max_steps + 1);
            time = zeros(1, max_steps + 1);

            pursuer_positions(:,1) = env.pursuer.position;
            for i=1:env.evader_numbers
                evader_positions(:,i,1) = env.evaders(i).position;
            end
            captured_evader_history(:,1) = env.captured_evaders';

            if plot_flag
                figure
                hold on
                grid on
                axis equal
                xlabel('x')
                ylabel('y')
                title('Pursuer-Evader Simulation')

                pursuer_traj_plot = plot(pursuer_positions(1,1), pursuer_positions(2,1), 'r-', 'LineWidth', 1.5);
                pursuer_current_plot = plot(pursuer_positions(1,1), pursuer_positions(2,1), 'r.', 'MarkerSize', 30);
                evader_traj_plots = gobjects(1, env.evader_numbers);
                evader_current_plots = gobjects(1, env.evader_numbers);
                for i=1:env.evader_numbers
                    evader_traj_plots(i) = plot(evader_positions(1,i,1), evader_positions(2,i,1), 'b-', 'LineWidth', 1.5);
                    evader_current_plots(i) = plot(evader_positions(1,i,1), evader_positions(2,i,1), 'b.', 'MarkerSize', 30);
                end
                plot(env.target_position(1), env.target_position(2), 'g.', 'MarkerSize', 30)
                drawnow
            end

            done = false;
            step_count = 0;
            while ~done && step_count < max_steps
                done = env.step();
                step_count = step_count + 1;

                time(step_count + 1) = step_count * env.timestep;
                pursuer_positions(:,step_count + 1) = env.pursuer.position;
                for i=1:env.evader_numbers
                    evader_positions(:,i,step_count + 1) = env.evaders(i).position;
                end
                captured_evader_history(:,step_count + 1) = env.captured_evaders';

                if plot_flag
                    set(pursuer_traj_plot, 'XData', pursuer_positions(1,1:step_count + 1), ...
                        'YData', pursuer_positions(2,1:step_count + 1));
                    set(pursuer_current_plot, 'XData', env.pursuer.position(1), ...
                        'YData', env.pursuer.position(2));
                    for i=1:env.evader_numbers
                        evader_traj = squeeze(evader_positions(:,i,1:step_count + 1));
                        set(evader_traj_plots(i), 'XData', evader_traj(1,:), ...
                            'YData', evader_traj(2,:));
                        set(evader_current_plots(i), 'XData', env.evaders(i).position(1), ...
                            'YData', env.evaders(i).position(2));
                    end
                    drawnow
                    pause(env.timestep)
                end
            end

            final_index = step_count + 1;
            history.time = time(1:final_index);
            history.pursuer_positions = pursuer_positions(:,1:final_index);
            history.evader_positions = evader_positions(:,:,1:final_index);
            history.captured_evaders = captured_evader_history(:,1:final_index);
            history.target_position = env.target_position;
            history.step_count = step_count;
            history.terminated = done;

            if done && all(env.captured_evaders)
                history.termination_reason = "all_evaders_captured";
            elseif done
                history.termination_reason = "evader_reached_target";
            else
                history.termination_reason = "max_steps_reached";
                warning("Simulation stopped after max_steps before termination.")
            end

            if plot_flag
                hold off
            end
        end

    end

end
