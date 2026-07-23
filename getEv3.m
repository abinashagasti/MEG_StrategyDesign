function ev3 = getEv3(serial, ip, reset)
% GETEV3  Returns a shared legoev3 connection for a brick, creating it once.
%
% An EV3 accepts only ONE connection at a time, and creating a legoev3 is slow.
% If calibrate_yaw_offset, closedloop_optitrack_ev3motion and env_hardware each
% opened their own connection to the same brick, the second would fail with
% "Failed to connect to EV3 through WiFi". Route every connection through here
% so each brick is opened exactly once per MATLAB session and reused.
%
%   ev3 = getEv3(serial, ip)         % create on first use, reuse afterwards
%   ev3 = getEv3(serial, ip, true)   % drop the cached connection and reconnect
%
% Connections live in a persistent map, surviving between runs; they are torn
% down by `clear getEv3` / `clear all`, or with reset=true.

    persistent bricks
    if isempty(bricks)
        bricks = containers.Map('KeyType','char','ValueType','any');
    end

    key = char(string(serial));

    if nargin >= 3 && ~isempty(reset) && reset && isKey(bricks, key)
        remove(bricks, key);   % drop our reference so the old socket is released
    end

    if isKey(bricks, key)
        ev3 = bricks(key);
        return
    end

    fprintf("Connecting to EV3 %s at %s ...\n", key, string(ip));
    ev3 = legoev3('wifi', char(string(ip)), key);
    bricks(key) = ev3;
end
