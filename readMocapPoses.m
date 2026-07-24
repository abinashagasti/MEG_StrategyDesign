function positions = readMocapPoses(node, names, up_axis)
% READMOCAPPOSES  One-shot read of several rigid bodies' floor positions.
%
%   positions = readMocapPoses(node, ["pursuer","evader1","evader2"])
%   positions = readMocapPoses(node, names, 'z')   % up_axis, default 'z'
%
% Returns a 2 x numel(names) matrix of floor positions (m), column k for
% names(k). `node` is a ros2node (e.g. from getMocapNode).
%
% Creates ALL subscribers first and keeps them alive together, with a short
% pause for DDS discovery before reading. Creating/destroying a subscriber per
% body in a loop churns the transport and throws "Transport stopped" -- this
% mirrors ros2_test.m's create-then-read pattern.
    if nargin < 3, up_axis = 'z'; end
    names = string(names);
    m     = numel(names);

    subs = cell(1, m);
    for k = 1:m
        topic   = "/vrpn_mocap/" + names(k) + "/pose";
        subs{k} = ros2subscriber(node, topic, "geometry_msgs/PoseStamped", ...
                                  "Reliability", "besteffort");
    end
    pause(0.5);   % let DDS discover the publishers before the first receive

    positions = zeros(2, m);
    for k = 1:m
        msg = receive(subs{k}, 3);
        positions(:,k) = readPose(msg, up_axis);   % readPose returns [pos, yaw, raw]
    end
end
