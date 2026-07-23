function node = getMocapNode(reset) %#ok<INUSD>
% GETMOCAPNODE  Returns a fresh ROS2 node for the mocap stream.
%
%   node = getMocapNode();
%
% A ros2node is simply created on every call. It is NOT persisted/reused:
% reusing a stale ros2node was found to CRASH MATLAB (a dead DDS participant
% still looks like a valid handle, and adding a subscriber to it faults the DDS
% layer -- a native crash that cannot be caught). Unlike a legoev3 connection,
% recreating "/matlab_node" does not error, so a fresh node each call is both
% safe and cheap.
%
% (The `reset` argument is accepted for call-site compatibility but ignored.)

    node = ros2node("/matlab_node");
end
