function [pos, yaw, raw] = readPose(msg, up_axis)
    % Returns position and orientation of the rigid body
    % tracked by msg. The up_axis is typically 'z'.

    % 3D position information given by optitrack
    raw = [msg.pose.position.x; msg.pose.position.y; msg.pose.position.z];
    % Orientation information given by optitrack in quaternions
    q   = [msg.pose.orientation.w, msg.pose.orientation.x, ...
           msg.pose.orientation.y, msg.pose.orientation.z];
    switch lower(up_axis)
        case 'z'
            % Only first two coordinates represent x and y axes
            pos = [raw(1); raw(2)];
            eul = quat2eul(q, 'ZYX');
            % Given the way q is written, the first euler angle 
            % gives the heading orientation on the 2D plane.
            yaw = eul(1);
        case 'y'
            pos = [raw(1); raw(3)];
            eul = quat2eul(q, 'YXZ');
            yaw = eul(1);
        otherwise
            error("UP_AXIS must be 'y' or 'z'.")
    end
end