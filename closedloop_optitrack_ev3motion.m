function closedloop_optitrack_ev3motion(goal)
% Drive a single EV3 kiwi-drive bot to a goal using OptiTrack feedback.
% NOTE: this is a FUNCTION (not a script) on purpose -- it makes the
% onCleanup motor-stop fire on ANY exit: normal finish, error, or Ctrl-C.
clc

%% ================= CONFIG =================
% --- EV3 connection ---
EV3_IP     = '192.168.0.154';
EV3_SERIAL = '00165347fbd9';

% --- OptiTrack / ROS2 ---
POSE_TOPIC = "/vrpn_mocap/pursuer/pose";

% --- Coordinate frame ---
UP_AXIS    = 'z';        % 'z' -> floor = (x,y);  'y' -> floor = (x,z)
% YAW_OFFSET = -1.4085;    % rad, measured via calibrate_yaw_offset.m (-80.7 deg)
YAW_OFFSET = -1.5;

% --- Robot parameters (kiwi / 3-wheel omni) ---
r = 0.025;               % wheel radius (m)
L = 0.14;                % centre-to-wheel distance (m)
J = (1/r) * [ ...
    -L      1        0;
    -L    -0.5   -sqrt(3)/2;
    -L    -0.5    sqrt(3)/2 ];

% --- Motor mapping ---
MOTOR_MAX_RADPS = 16;    % wheel rad/s at 100% (lower this to move faster)
MOTOR_CLAMP     = 90;

% --- Control law ---
Kp   = 1.0;              % proportional position gain
vmax = 0.15;             % max translational speed (m/s)
tol  = 0.07;             % stop radius (m)
dt   = 0.05;             % control period (s)
timeout = 60;            % abort after this many seconds

% --- Mocap dropout handling ---
RECEIVE_TIMEOUT = 2;     % s to wait for each pose message
MAX_MISS        = 10;    % consecutive misses before aborting

% Goal -- accept a row or column, force to a 2x1 column vector.
if nargin < 1 || isempty(goal)
    goal = [0;0];
end
goal = goal(:);
if numel(goal) ~= 2 || ~all(isfinite(goal))
    error("goal must be a 2-element finite vector, e.g. [0.5; 0.3].");
end

%% ================= CONNECT =================
myev3 = legoev3('wifi', EV3_IP, EV3_SERIAL);
mA = motor(myev3,'A');
mB = motor(myev3,'B');
mC = motor(myev3,'C');

% Local onCleanup -> fires on error, Ctrl-C, or normal return.
% Registered right after the motors exist so any later failure stops them.
cleanup = onCleanup(@() stopMotors(mA,mB,mC)); %#ok<NASGU>

node = ros2node("/matlab_node");
sub  = ros2subscriber(node, POSE_TOPIC, "geometry_msgs/PoseStamped", ...
                      "Reliability","besteffort");

%% ================= INITIAL POSE =================
msg = receive(sub, RECEIVE_TIMEOUT);
[pos0, yaw0, raw0] = readPose(msg, UP_AXIS);
fprintf("Raw position (x,y,z) = %.3f, %.3f, %.3f\n", raw0(1), raw0(2), raw0(3));
fprintf("Floor position       = %.3f, %.3f   yaw = %.1f deg\n", ...
        pos0(1), pos0(2), rad2deg(yaw0));
fprintf("Goal                 = %.3f, %.3f   (%.2f m away)\n", ...
        goal(1), goal(2), norm(goal - pos0));
disp("Starting in 2 s... (Ctrl-C to abort; motors stop safely)")
pause(2)

%% ================= CONTROL LOOP =================
start(mA); start(mB); start(mC);
t_start = tic;
miss = 0;

while true
    % --- Robust pose read: a dropped frame stops the bot and retries,
    %     rather than crashing the loop with the motors still running. ---
    try
        msg = receive(sub, RECEIVE_TIMEOUT);
    catch
        miss = miss + 1;
        stopMotors(mA,mB,mC);              % never drive on stale data
        warning("Missed mocap frame (%d/%d).", miss, MAX_MISS);
        if miss >= MAX_MISS
            error("Lost mocap stream (rigid body occluded?). Aborting.");
        end
        continue
    end
    miss = 0;

    [pos, yaw] = readPose(msg, UP_AXIS);
    err  = goal - pos;
    dist = norm(err);
    fprintf("pos %.3f %.3f | yaw %+.0f deg | err %.3f\n", ...
            pos(1), pos(2), rad2deg(yaw), dist);

    if dist < tol
        disp("Goal reached.")
        break
    end
    if toc(t_start) > timeout
        warning("Timeout before reaching goal.")
        break
    end

    % Proportional velocity in WORLD frame, saturated.
    vel = Kp*err;
    sp  = norm(vel);
    if sp > vmax
        vel = vel/sp*vmax;
    end

    % Rotate WORLD velocity into ROBOT BODY frame.
    c = cos(yaw + YAW_OFFSET); s = sin(yaw + YAW_OFFSET);
    R_w2b = [ c  s; -s  c];
    v_body = R_w2b*vel;          % [vx (forward); vy (left)]
    omega  = 0;

    % Inverse kinematics -> wheel speeds -> motor percent.
    wheelOmega = J*[omega; v_body(1); v_body(2)];
    motorPct   = 100*wheelOmega/MOTOR_MAX_RADPS;
    motorPct   = max(min(motorPct, MOTOR_CLAMP), -MOTOR_CLAMP);

    % Wheel-to-motor mapping (wheel1->C, wheel2->A, wheel3->B)
    mA.Speed = motorPct(2);
    mB.Speed = motorPct(3);
    mC.Speed = motorPct(1);

    pause(dt)
end

stopMotors(mA,mB,mC);
disp("Done.")
end

%% ================= HELPERS =================
function stopMotors(mA,mB,mC)
    try, stop(mA); end %#ok<*TRYNC>
    try, stop(mB); end
    try, stop(mC); end
end
