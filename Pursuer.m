classdef Pursuer < handle
    properties (SetAccess = public)
        position % 2x1 vector representing current position
        speed % scalar representing maximum speed
        motor1 % motor variables 
        motor2
        motor3
        pos_vrpn
        ori_vrpn
        wheel_radius
        wheel_centre_radius
        convex_optimization_flag % if the optimization in non-heuristic 
        % methods is over the subset of the pareto optimal set where all
        % di functions are concave
    end

    methods

        function p = Pursuer(initPos, speed, convex_optimization_flag)
            % Constructor, assigns initial values
            p.position = initPos;
            p.speed = speed;
            p.wheel_radius = 0.025;
            p.wheel_centre_radius = 0.15;
            p.convex_optimization_flag = convex_optimization_flag;
        end
        
        function updatePos(p, position)
            % Updates position and velocity based on evader position
            sz_input = size(position);
            sz_output = size(p.position);
            if sz_input~=sz_output
                if sz_input==fliplr(sz_output)
                    p.position = position';
                else
                    error('Wrong position dimensions.')
                end
            else
                p.position = position;
            end
        end

        function position = getPos(p)
            % Returns position of pursuer
            position = p.position;
        end

        function [psi_star, theta_star, m, xm, ym] = optimal_headings_Ei(p, evader_position, target_position)
            % Determine evader and pursuer 1v1 optimal headings
            relative_position = evader_position - p.position;
            midpoint = 0.5*(evader_position + p.position);
            intercept = target_position - relative_position*((target_position - midpoint)'*relative_position)/(relative_position'*relative_position);
            xI = intercept(1);
            yI = intercept(2);
            m = relative_position(2)/relative_position(1);
            xm = midpoint(1);
            ym = midpoint(2);

            psi_star = atan2(yI-evader_position(2),xI-evader_position(1));
            theta_star = atan2(yI-p.position(2),xI-p.position(1));
        end

        function [velocity,theta] = optimal_pursuer_heading(p, evader_positions, target_position, timestep, win)
            shape = size(evader_positions);
            n = shape(2);
            psi_values = zeros(1,n);
            theta_values = zeros(1,n);
            evader_next_step_positions = zeros(2,n);
            closest_evader_distance = Inf;
            if win
                for i=1:n
                    [psi_star, theta_star] = p.optimal_headings_Ei(evader_positions(:,i), target_position);
                    psi_values(i) = psi_star;
                    theta_values(i) = theta_star;
                    evader_next_step_positions(:,i) = evader_positions(:,i) + timestep * [cos(psi_star);sin(psi_star)];
                    if norm(evader_next_step_positions(:,i)-target_position)<closest_evader_distance
                        closest_evader_distance = norm(evader_next_step_positions(:,i)-target_position);
                        closest_evader = i;
                    end
                end            
                theta = theta_values(closest_evader);
                velocity = p.speed*[cos(theta);sin(theta)];
            else
                velocity = target_position - p.position;
                velocity = (p.speed/norm(velocity))*velocity;
                theta = atan2(velocity(2),velocity(1));
            end
                
        end

        function d_i = objective_Ei(p, evader_position, target_position, r, theta)
            [psi_star, ~] = p.optimal_headings_Ei(evader_position, target_position);
            relative_position = evader_position - p.position;
            midpoint = 0.5*(evader_position + p.position);
            line_normal = [-relative_position(2); relative_position(1)];
            Tx = abs((target_position - midpoint)'*relative_position)/norm(relative_position);
            Ty = abs((evader_position - target_position)'*line_normal)/norm(relative_position);
            k = norm(p.position-evader_position)/2;
            delta = atan2(p.position(2)-evader_position(2),p.position(1)-evader_position(1));
            d_i = Tx + (r/(2*k))*sqrt(Ty^2+k^2)*(-1-cos(theta+psi_star-2*delta));
        end

        function [theta_min, theta_max, min_evader, max_evader] = concave_domain(p, evader_positions, target_position)
            shape = size(evader_positions);
            n = shape(2);
            theta_keys = 1:n;
            theta_values = zeros(1,n);
            for i=1:n
                [~,theta_star] = p.optimal_headings_Ei(evader_positions(:,i), target_position);
                theta_values(i) = theta_star;
            end
            [theta_values, sort_order] = sort(theta_values);
            theta_keys = theta_keys(sort_order);
            [theta_largest, max_evader] = max(theta_values);
            [theta_smallest, min_evader] = min(theta_values);
            max_evader = theta_keys(max_evader);
            min_evader = theta_keys(min_evader);
            if p.convex_optimization_flag
                theta_min = max(theta_smallest, theta_largest-pi/2);
                theta_max = min(theta_largest, theta_smallest+pi/2);
            else
                theta_min = theta_smallest;
                theta_max = theta_largest;
            end
        end

        function cost = objective_fun(p, evader_positions, target_position, r, theta)
            shape = size(evader_positions);
            cost = 0;
            for i=1:shape(2)
                cost = cost + 1/(p.objective_Ei(evader_positions(:,i),target_position,r,theta));
            end
        end

        function cost = objective_fun_squaresum(p, evader_positions, target_position, r, theta)
            shape = size(evader_positions);
            cost = 0;
            for i=1:shape(2)
                cost = cost + (p.objective_Ei(evader_positions(:,i),target_position,r,theta))^2;
            end
        end

        function cost = objective_fun_squaresump(p, evader_positions, target_position, r, theta)
            shape = size(evader_positions);
            cost = 0;
            for i=1:shape(2)
                cost = cost + 1/(p.objective_Ei(evader_positions(:,i),target_position,r,theta))^2;
            end
        end

        function theta = heading_direction(p, evader_positions, target_position, r)
            cost = @(theta) p.objective_fun(evader_positions, target_position, r, theta);
            theta = p.optimize_heading(cost, evader_positions, target_position);
        end

        function theta = heading_direction_squaresum(p, evader_positions, target_position, r)
            cost = @(theta) -p.objective_fun_squaresum(evader_positions, target_position, r, theta);
            theta = p.optimize_heading(cost, evader_positions, target_position);
        end

        function theta = heading_direction_squaresump(p, evader_positions, target_position, r)
            cost = @(theta) p.objective_fun_squaresump(evader_positions, target_position, r, theta);
            theta = p.optimize_heading(cost, evader_positions, target_position);
        end

        function theta = optimize_heading(p, cost, evader_positions, target_position)
            [theta_min, theta_max] = p.concave_domain(evader_positions, target_position);
            if theta_min == theta_max
                theta = theta_min;
                return
            end
            options = optimoptions("fmincon",...
                    "Algorithm","interior-point",...
                    "EnableFeasibilityMode",true,...
                    "SubproblemAlgorithm","cg", "Display","none");
            theta = fmincon(cost,0.5*(theta_min+theta_max),[],[],[],[],theta_min,theta_max,[],options);
        end

        function theta = heading_direction_heuristic(p, evader_positions, target_position, r)
            shape = size(evader_positions);
            p_values = zeros(1,shape(2));
            theta_star = zeros(1,shape(2));
            for i=1:shape(2)
                [~,theta_star(i)]=p.optimal_headings_Ei(evader_positions(:,i),target_position);
            end
            for i=1:shape(2)
                p_values(i) = p.objective_Ei(evader_positions(:,i), target_position, r, theta_star(i));
            end
            weights = p_values/sum(p_values);
            theta = weights*theta_star';
        end

        function theta = heading_direction_closest(p, evader_positions)
            shape = size(evader_positions);
            dist = Inf;
            imin = 0;
            for i=1:shape(2)
                if dist>norm(p.position-evader_positions(:,i))
                    dist = norm(p.position-evader_positions(:,i));
                    imin = i;
                end
            end
            theta = atan2(evader_positions(2,imin)-p.position(2),evader_positions(1,imin)-p.position(1));
        end

        function [velocity, theta] = heading_velocity(p, evader_positions, target_position, timestep, win, pursuer_policy)
            if ~exist('pursuer_policy','var') || isempty(pursuer_policy)
                pursuer_policy = "closest_next_step";
            end
            pursuer_policy = string(pursuer_policy);
            if win
                if matches(pursuer_policy,"closest_next_step")
                    [velocity, theta] = p.optimal_pursuer_heading(evader_positions, target_position, timestep, win);
                    return
                elseif matches(pursuer_policy,"standard")
                    old_convex_optimization_flag = p.convex_optimization_flag;
                    p.convex_optimization_flag = true;
                    theta = p.heading_direction(evader_positions, target_position, timestep);
                    p.convex_optimization_flag = old_convex_optimization_flag;
                elseif matches(pursuer_policy,"squaresum")
                    old_convex_optimization_flag = p.convex_optimization_flag;
                    p.convex_optimization_flag = true;
                    theta = p.heading_direction_squaresum(evader_positions, target_position, timestep);
                    p.convex_optimization_flag = old_convex_optimization_flag;
                elseif matches(pursuer_policy,"squaresump")
                    old_convex_optimization_flag = p.convex_optimization_flag;
                    p.convex_optimization_flag = true;
                    theta = p.heading_direction_squaresump(evader_positions, target_position, timestep);
                    p.convex_optimization_flag = old_convex_optimization_flag;
                elseif matches(pursuer_policy,"heuristic")
                    theta = p.heading_direction_heuristic(evader_positions, target_position, timestep);
                elseif matches(pursuer_policy,"closest")
                    theta = p.heading_direction_closest(evader_positions);
                else
                    error("Unknown pursuer policy. Choose closest_next_step, standard, squaresum, squaresump, heuristic, or closest.")
                end
                velocity = p.speed*[cos(theta);sin(theta)];
            else
                velocity = target_position - p.position;
                velocity = (p.speed/norm(velocity))*velocity;
                theta = atan2(velocity(2),velocity(1));
            end
        end

        function init_start_mtr(p,myev3)
            % Start the pursuer motors
            p.motor1 = motor(myev3,'A');
            p.motor2 = motor(myev3,'B');
            p.motor3 = motor(myev3,'C');
            start(p.motor1);
            start(p.motor2);
            start(p.motor3);
        end

        function stop_mtr(p)
            % Stop the pursuer motors
            stop(p.motor1);
            stop(p.motor2);
            stop(p.motor3);
        end

        function set_mtr_speed(p,speed)
            % Set pursuer motor speed
            p.motor1.Speed = speed(1);
            p.motor1.Speed = speed(2);
            p.motor1.Speed = speed(3);
        end

        function callback(p,message)
            disp("poda")
            p.pos_vrpn =[message.Pose.Position.X message.Pose.Position.Y message.Pose.Position.Z];
            p.ori_vrpn = [message.Pose.Orientation.X message.Pose.Orientation.Y message.Pose.Orientation.Z message.Pose.Orientation.W]; 
        end
    end
end
