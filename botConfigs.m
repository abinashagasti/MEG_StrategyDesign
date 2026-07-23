function cfg = botConfigs(name)
% BOTCONFIGS  Single source of truth for the per-bot hardware constants.
%
%   cfg = botConfigs()           % struct of ALL bots
%   cfg = botConfigs("evader1")  % one bot's config (a BotHardware config struct)
%
% Everything here is a property of the INDIVIDUAL robot, not a shared constant:
% the bots differ in wheel geometry, motor response and yaw offset.
%
% Fields (see BotHardware.CONFIG_FIELDS):
%   name             mocap rigid body name -> topic /vrpn_mocap/<name>/pose
%   ev3_ip           EV3 wifi address (DHCP: re-check on the brick if it changes)
%   ev3_serial       EV3 serial number
%   wheel_radius     r (m)
%   wheel_offset     L, centre-to-wheel (m)
%   yaw_offset       rad, from calibrate_yaw_offset(<name>)
%   motor_max_radps  wheel rad/s at 100% duty, from calibrate_yaw_offset(<name>)
%
% NOTE: yaw_offset is measured against the CURRENT Motive ground plane and the
% CURRENT rigid body definition. Re-wanding, re-setting the ground plane, or
% recreating a rigid body INVALIDATES it -- re-run the calibration.

    bots = struct();

    bots.pursuer = struct( ...
        'name',            "pursuer", ...
        'ev3_ip',          "192.168.0.154", ...
        'ev3_serial',      "00165347fbd9", ...
        'wheel_radius',    0.025, ...
        'wheel_offset',    0.14, ...
        'yaw_offset',      -1.5923, ...   % TODO: stale after the re-wand -- re-measure
        'motor_max_radps', 8.5);       % TODO: measured ~47% of commanded -> expect ~7.5

    bots.evader1 = struct( ...
        'name',            "evader1", ...
        'ev3_ip',          "192.168.0.108", ...
        'ev3_serial',      "00165347c4f7", ....% "00165347593f", ...
        'wheel_radius',    0.025, ...  % TODO: measure this bot's actual wheel
        'wheel_offset',    0.14, ...   % TODO: measure centre-to-wheel
        'yaw_offset',      -1.6272, ...    % TODO: run calibrate_yaw_offset("evader1")
        'motor_max_radps', 8.5);

    bots.evader2 = struct( ...
        'name',            "evader2", ...
        'ev3_ip',          "192.168.0.197", ...
        'ev3_serial',      "00165348d9ee", ...
        'wheel_radius',    0.025, ...  % TODO: measure this bot's actual wheel
        'wheel_offset',    0.14, ...   % TODO: measure centre-to-wheel
        'yaw_offset',      -0.9078, ...    % TODO: run calibrate_yaw_offset("evader2")
        'motor_max_radps', 12);

    if nargin < 1 || isempty(name)
        cfg = bots;
        return
    end

    key = char(string(name));
    if ~isfield(bots, key)
        error("botConfigs:unknownBot", "Unknown bot '%s'. Known bots: %s.", ...
              key, strjoin(string(fieldnames(bots))', ", "));
    end
    cfg = bots.(key);
end
