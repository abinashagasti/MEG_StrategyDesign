
% % Use the following commands on terminal
% ros2 launch vrpn_mocap client.launch.yaml server:=192.168.0.118
% check the list of topics in ros2 topic list

% Create a ROS 2 node

node = ros2node("/matlab_node");

sub = ros2subscriber( ...
    node, ...
    "/vrpn_mocap/pursuer/pose", ...
    "geometry_msgs/PoseStamped", ...
    "Reliability","besteffort");

%%

msg = receive(sub, 2);
position = [msg.pose.position.x;msg.pose.position.y;msg.pose.position.z]

ori = [msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z];
eul = quat2eul(ori,'ZYX')'