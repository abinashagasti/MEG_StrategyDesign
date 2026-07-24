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
%   wheel_radius     r (m)  -- nominal; see note below
%   wheel_offset     L, centre-to-wheel (m) -- nominal; see note below
%   yaw_offset       rad, from calibrate_yaw_offset(<name>)
%   motor_max_radps  wheel rad/s at 100% duty, from calibrate_yaw_offset(<name>)
% Optional (BotHardware defaults in brackets): motor_deadband [0], motor_clamp
% [90], wheel_ports [["C","A","B"]], up_axis ['z'].
%
% NOTES
%  - yaw_offset is tied to the CURRENT Motive ground plane and rigid body
%    definition. Re-wanding, re-setting the ground plane, or recreating a rigid
%    body INVALIDATES it -- re-run calibrate_yaw_offset for that bot.
%  - motor_max_radps drifts with BATTERY level (open-loop). For a fair game keep
%    the bots at similar charge; recalibrate at session start if you want it
%    tight. It is calibrated at push_speed = the operating speed (currently 0.18).
%  - wheel_radius / wheel_offset are NOMINAL placeholders (not per-bot measured).
%    That's fine: for pure translation the radius error is absorbed into the
%    calibrated motor_max_radps. Only measure them per bot if you reintroduce a
%    heading (omega) controller, which uses the L/r ratio directly.

    bots = struct();

    bots.pursuer = struct( ...
        'name',            "pursuer", ...
        'ev3_ip',          "192.168.0.154", ...
        'ev3_serial',      "00165347fbd9", ...
        'wheel_radius',    0.025, ...      % nominal
        'wheel_offset',    0.14, ...       % nominal
        'yaw_offset',      -1.5923, ...    % calibrated
        'motor_max_radps', 8);            % calibrated

    bots.evader1 = struct( ...
        'name',            "evader1", ...
        'ev3_ip',          "192.168.0.108", ...
        'ev3_serial',      "00165347c4f7", ...  % (was 00165347593f before brick swap)
        'wheel_radius',    0.025, ...      % nominal
        'wheel_offset',    0.14, ...       % nominal
        'yaw_offset',      -1.6272, ...    % calibrated
        'motor_max_radps', 9.2);          % calibrated

    bots.evader2 = struct( ...
        'name',            "evader2", ...
        'ev3_ip',          "192.168.0.197", ...
        'ev3_serial',      "00165348d9ee", ...
        'wheel_radius',    0.025, ...      % nominal
        'wheel_offset',    0.14, ...       % nominal
        'yaw_offset',      -1.547, ...     % calibrated
        'motor_max_radps', 11.4);         % calibrated

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
