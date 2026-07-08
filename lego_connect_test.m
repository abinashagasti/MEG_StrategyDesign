clear
clc

%% Connect to EV3
% myev3 = legoev3('wifi','192.168.0.154','00165347593f');
myev3 = legoev3('wifi','192.168.0.154','00165347fbd9');

%% Create motor objects
mA = motor(myev3,'A');   % Left wheel
mB = motor(myev3,'B');   % Right wheel
mC = motor(myev3,'C');   % Back wheel

%% Desired robot velocity
% Robot frame:
% vx     : forward (+)
% vy     : left (+)
% omega  : counter-clockwise (+)

vx = 0.20;      % m/s
vy = 0.00;      % m/s
omega = 0.00;   % rad/s

% Robot parameters
r = 0.025;      % Wheel radius (m)
L = 0.14;      % Distance from robot center to wheel (m)

% Inverse kinematics (Kiwi drive)
J = (1/r) * [
    -L      1       0;
    -L    -0.5   -sqrt(3)/2;
    -L    -0.5    sqrt(3)/2
];

wheelOmega = J * [omega; vx; vy];

% Convert wheel angular velocity to EV3 motor commands
% Gain to map rad/s to EV3 Speed (-100 to 100)
gain = 10;

motorSpeed = gain * wheelOmega;

% Saturate commands
motorSpeed = max(min(motorSpeed,90),-90);

%% Assign motor speeds
motorSpeed = 50*[-1;0.5;0.5];

mA.Speed = motorSpeed(2);
mB.Speed = motorSpeed(3);
mC.Speed = motorSpeed(1);

% Start robot
start(mA);
start(mB);
start(mC);

pause(5);

% Stop robot
stop(mA);
stop(mB);
stop(mC);