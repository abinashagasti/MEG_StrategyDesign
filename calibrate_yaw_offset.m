clear
clc

% CALIBRATE_YAW_OFFSET
% Measures the constant angle between the robot's body "forward" (the +vx
% axis assumed by the wheel Jacobian J) and the mocap yaw=0 reference.
%
% Method: command a PURE body-forward push, measure the direction the robot
% actually travels in the world frame, and compare it to the mocap heading.
%   YAW_OFFSET = wrapToPi(world_move_direction - mocap_yaw)
% Copy the printed value into closedloop_optitrack_ev3motion.m.
%
% Prereq: confirm UP_AXIS first (the "slide the bot" test). If the plane is
% wrong, this number is meaningless.

%% ================= CONFIG (keep in sync with the main script) =================
EV3_IP     = '192.168.0.154';
EV3_SERIAL = '00165347fbd9';
POSE_TOPIC = "/vrpn_mocap/pursuer/pose";

UP_AXIS = 'z';               % 'z' -> floor=(x,y);  'y' -> floor=(x,z)

r = 0.025;                   % wheel radius (m)
L = 0.14;                    % centre-to-wheel distance (m)
J = (1/r) * [ ...
    -L      1        0;
    -L    -0.5   -sqrt(3)/2;
    -L    -0.5    sqrt(3)/2 ];

MOTOR_MAX_RADPS = 16;        % wheel rad/s at 100%
MOTOR_CLAMP     = 90;

% --- Calibration motion ---
PUSH_SPEED = 0.15;           % body forward speed during a push (m/s)
PUSH_TIME  = 1.5;            % how long to push (s)
NREPS      = 3;              % number of pushes to average
MIN_DISP   = 0.05;           % warn if a push moves less than this (m)
MAX_SPIN   = 20;             % deg; exclude a push that rotates more than this

%% ================= CONNECT =================
myev3 = legoev3('wifi', EV3_IP, EV3_SERIAL);
mA = motor(myev3,'A');
mB = motor(myev3,'B');
mC = motor(myev3,'C');

node = ros2node("/matlab_node");
sub  = ros2subscriber(node, POSE_TOPIC, "geometry_msgs/PoseStamped", ...
                      "Reliability","besteffort");

cleanup = onCleanup(@() stopMotors(mA,mB,mC));

%% ================= RUN PUSHES =================
disp("Give the bot ~1 m of clear space ahead. Starting in 3 s...")
pause(3)

offsets = nan(1,NREPS);         % NaN = rep excluded from the average
for k = 1:NREPS
    fprintf("\n--- Push %d/%d ---\n", k, NREPS);

    msg = receive(sub, 3);
    [p0, yaw0] = readPose(msg, UP_AXIS);

    % Command PURE body-forward: [omega; vx; vy] = [0; PUSH_SPEED; 0]
    wheelOmega = J*[0; PUSH_SPEED; 0];
    motorPct   = 100*wheelOmega/MOTOR_MAX_RADPS;
    motorPct   = max(min(motorPct, MOTOR_CLAMP), -MOTOR_CLAMP);
    mA.Speed = motorPct(2);
    mB.Speed = motorPct(3);
    mC.Speed = motorPct(1);
    start(mA); start(mB); start(mC);

    % SAMPLE the pose throughout the push, so we can use the MEAN heading
    % (the displacement direction matches the average yaw, not the start).
    yaws = yaw0;
    p1   = p0;
    t0   = tic;
    while toc(t0) < PUSH_TIME
        try
            msg = receive(sub, 2);
            [p1, y] = readPose(msg, UP_AXIS);
            yaws(end+1) = y; %#ok<AGROW>
        catch
            % ignore an isolated dropped frame during calibration
        end
        pause(0.05);
    end
    stopMotors(mA,mB,mC);
    pause(0.5);              % let it settle, then take the final reading
    try
        msg = receive(sub, 2);
        [p1, yaw1] = readPose(msg, UP_AXIS);
        yaws(end+1) = yaw1;
    catch
        yaw1 = yaws(end);
    end

    d        = p1 - p0;
    disp_mag = norm(d);
    move_dir = atan2(d(2), d(1));
    mean_yaw = atan2(mean(sin(yaws)), mean(cos(yaws)));   % circular mean
    net_spin = wrapToPi(yaw1 - yaw0);
    offsets(k) = wrapToPi(move_dir - mean_yaw);           % use MEAN yaw

    fprintf("  moved %.3f m | world dir %+.1f | mean yaw %+.1f | spin %+.1f | offset %+.1f (deg)\n", ...
            disp_mag, rad2deg(move_dir), rad2deg(mean_yaw), rad2deg(net_spin), rad2deg(offsets(k)));

    if disp_mag < MIN_DISP
        warning("  Tiny displacement (%.3f m). Check UP_AXIS, motor mapping, or wiring.", disp_mag);
    end
    if abs(net_spin) > deg2rad(MAX_SPIN)
        warning("  Spun %.1f deg (> %d) during the push -> excluding this rep from the average.", ...
                rad2deg(net_spin), MAX_SPIN);
        offsets(k) = NaN;
    end
end

%% ================= RESULT =================
% Average only the reps that did not spin too much.
valid = offsets(~isnan(offsets));
if isempty(valid)
    stopMotors(mA,mB,mC);
    error(['Every push spun more than %d deg -- no trustworthy reading. ' ...
           'Reduce PUSH_TIME/PUSH_SPEED, or fix motor asymmetry (heading-hold).'], MAX_SPIN);
end
% Circular mean (offsets are angles).
mean_offset = atan2(mean(sin(valid)), mean(cos(valid)));
spread      = rad2deg(max(valid) - min(valid));

fprintf("\n==================================================\n");
fprintf("Measured YAW_OFFSET = %.4f rad  (%.1f deg)\n", mean_offset, rad2deg(mean_offset));
fprintf("Averaged %d of %d pushes | spread = %.1f deg", numel(valid), NREPS, spread);
if spread > 20
    fprintf("   <-- HIGH: readings inconsistent, recheck UP_AXIS / slippage\n");
else
    fprintf("   (consistent)\n");
end
fprintf("Set in closedloop_optitrack_ev3motion.m:\n");
fprintf("    YAW_OFFSET = %.4f;\n", mean_offset);
fprintf("==================================================\n");

fprintf("\nConsistency check (optional): rotate the bot ~90 deg by hand and\n");
fprintf("rerun. The measured YAW_OFFSET should stay about the SAME. If it\n");
fprintf("moves with the bot, mocap yaw is not about the up-axis -> fix UP_AXIS.\n");

stopMotors(mA,mB,mC);

%% ================= HELPERS =================


function stopMotors(mA,mB,mC)
    try, stop(mA); end
    try, stop(mB); end
    try, stop(mC); end
end
