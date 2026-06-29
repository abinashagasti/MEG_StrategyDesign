clear
clc

% Connect to EV3
% myev3 = legoev3('wifi','192.168.0.154','00165347593f');
myev3 = legoev3('wifi','192.168.0.154','00165347fbd9');

%%

% Create motor objects
leftMotor  = motor(myev3,'A');
rightMotor = motor(myev3,'B');
backMotor  = motor(myev3,'C');

% Desired forward speed
V = 70;

% Wheel speeds (Kiwi drive)
leftMotor.Speed  = -0.5*V;
rightMotor.Speed = V;
backMotor.Speed  = 0;

% Start all motors
start(leftMotor);
start(rightMotor);
start(backMotor);

% Drive for 3 seconds
pause(10);

% Stop
stop(leftMotor);
stop(rightMotor);
stop(backMotor);

%%

% % Create motor objects
% m1 = motor(myev3, 'A');
% m2 = motor(myev3, 'B');
% m3 = motor(myev3, 'C');
% 
% % Reset encoders
% resetRotation(m1);
% resetRotation(m2);
% resetRotation(m3);
% 
% % Start motors
% start(m1);
% start(m2);
% start(m3);
% 
% %% 
% 
% % Straight-line motion command
% % For the omni-wheel geometry used in your code:
% % robot_twist = [omega; vx; vy]
% % omega = 0, vx = forward speed, vy = sideways speed
% vx = 30;
% vy = 0;
% omega = 0;
% 
% wheel_radius = 0.025;
% wheel_center_radius = 0.135;
% 
% J = (1 / wheel_radius) * [
%     -wheel_center_radius   1      0;
%     -wheel_center_radius  -0.5   -sin(pi/3);
%     -wheel_center_radius  -0.5    sin(pi/3)
% ];
% 
% wheel_cmd = J * [omega; vx; vy];
% 
% % Scale down because EV3 motor.Speed expects roughly -100 to 100
% gain = 0.02;
% motor_speed = gain * wheel_cmd;
% 
% % Clip motor commands
% motor_speed = max(min(motor_speed, 90), -90);
% 
% %%
% 
% % Send motor commands
% m1.Speed = motor_speed(1);
% m2.Speed = motor_speed(2);
% m3.Speed = motor_speed(3);
% 
% pause(2);
% 
% % Stop motors
% stop(m1);
% stop(m2);
% stop(m3);