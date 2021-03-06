%%

clc;
clear;
close all;

cd '~/catkin_ws/src/online_planner/rosbag'
savedir = 'anal_data/';

mkdir(savedir)
format long;

%%

filename = "vio_test_2021-09-15-15-10-52.bag";
bag = rosbag(filename);
bag.AvailableTopics

%%
vio_topic_name = "/vins_estimator/odometry";
cmd_name = "/airsim_node/SimpleFlight/rpy_or_rate_throttle";
traj_name = "/maptest_node/reference_trajectory";

topic_names = [vio_topic_name, cmd_name, traj_name];

bags = {};
for i=1:3
    bags{i} = select(bag, 'Topic', topic_names(i));
end

%% read messages

msgs = {};
for i=1:3
   msgs{i} =  readMessages(bags{i}, 'DataFormat', 'struct');
end

vio_msgs = msgs{1}; cmd_msgs = msgs{2}; traj_msgs = msgs{3};
len_vio = length(vio_msgs); len_cmd = length(cmd_msgs); len_traj = length(traj_msgs);
%%

vio_pos_arr = zeros(len_vio, 3);
vio_ori_arr = zeros(len_vio, 4);
vio_vel_arr = zeros(len_vio, 3);
for i=1:len_vio
   msg = vio_msgs(i);
   msg = msg{1};
   vio_pos_arr(i, 1) = msg.Pose.Pose.Position.X;
   vio_pos_arr(i, 2) = msg.Pose.Pose.Position.Y;
   vio_pos_arr(i, 3) = msg.Pose.Pose.Position.Z;
   vio_ori_arr(i, 1) = msg.Pose.Pose.Orientation.X;
   vio_ori_arr(i, 2) = msg.Pose.Pose.Orientation.Y;
   vio_ori_arr(i, 3) = msg.Pose.Pose.Orientation.Z;
   vio_ori_arr(i, 4) = msg.Pose.Pose.Orientation.W;
   vio_vel_arr(i, 1) = msg.Twist.Twist.Linear.X;
   vio_vel_arr(i, 2) = msg.Twist.Twist.Linear.Y;
   vio_vel_arr(i, 3) = msg.Twist.Twist.Linear.Z;
end

%%
figure(1);
for i=1:3
    subplot(3, 1, i);
    plot(vio_pos_arr(:,i));
end

figure(2);
for i=1:3
   subplot(3, 1, i);
   plot(vio_vel_arr(:, i));
end

%% 
ref_traj = msgs{3};
ref_traj_msg = ref_traj{1};

setpoints = ref_traj_msg.Setpoints;
num_setpoints = length(setpoints);
pos = zeros(3, num_setpoints);
yaw = zeros(1, num_setpoints);
vel = zeros(3, num_setpoints);
acc = zeros(3, num_setpoints);
T = zeros(1, num_setpoints);

for i=1:num_setpoints
    pos(1, i) = setpoints(i).Position.X;
    pos(2, i) = setpoints(i).Position.Y;
    pos(3, i) = setpoints(i).Position.Z;
    vel(1, i) = setpoints(i).Velocity.X;
    vel(2, i) = setpoints(i).Velocity.Y;
    vel(3, i) = setpoints(i).Velocity.Z;
    acc(1, i) = setpoints(i).AccelerationOrForce.X;
    acc(2, i) = setpoints(i).AccelerationOrForce.Y;
    acc(3, i) = setpoints(i).AccelerationOrForce.Z;
    yaw(i) = setpoints(i).Yaw;
    T(i) = get_time(setpoints(i)) - get_time(setpoints(1));
end

figure(1);
title("position reference trajectory"); hold on;
for i=1:3
    subplot(3, 1, i);
    plot(T, pos(i, :), "--b.", 'MarkerSize', 15); hold on;
end

figure(2);
title("Velocity reference trajectory"); hold on;
for i=1:3
   subplot(3, 1, i);
   plot(T, vel(i, :), "--b.", 'MarkerSize', 15); hold on;
end

figure(3);
title("Acceleration reference trajectory"); hold on;
for i=1:3
   subplot(3, 1, i);
   plot(T, acc(i, :), "--b.", 'MarkerSize', 15); hold on;
end 

figure(4);
title("yaw reference trajectory"); hold on;
plot(T, yaw, "--b.", 'MarkerSize', 15); hold off;


%%
function time_ = get_time(msg)
    time_ = double(msg.Header.Stamp.Sec) + 1e-9*double(msg.Header.Stamp.Nsec);
end