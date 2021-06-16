clear all;
close all;
%%
rosshutdown;
rosinit;
% roslaunch rst_mobile_robots turtlebot_in_stage.launch map_file:=obstacles_map move_base:=false amcl:=true

%%
global tftree;
tftree = rostf();
navGoalSub=rossubscriber('/move_base_simple/goal');
velPub =rospublisher('/mobile_base/commands/velocity');
disp('select navgoal in RVIZ to start obstacle avoidance');
pose = PoseHandle;
%odomSub = rossubscriber('/odom',{@odomCallback,pose});
%%

scanSubCallback = rossubscriber('/scan',{@scanCallback, velPub, navGoalSub, tftree, pose});


%% extra tests
scanSub = rossubscriber('/scan');
maxTime=1500;
rateObj=robotics.Rate(10);
rateObj.reset;
global x_recorded y_recorded t_recorded theta_recorded steeringDir v_recorded omega_recorded targetDir i;
i = 1;
odomSub =rossubscriber('/odom');
%%
while rateObj.TotalElapsedTime<maxTime
scanMsg = scanSub.LatestMessage;
scanCallback1(scanMsg, velPub, navGoalSub, tftree, pose);


[x_recorded(i), y_recorded(i), theta_recorded(i)]=OdometryMsg2Pose(odomSub.LatestMessage);
t_recorded(i) = rateObj.TotalElapsedTime;
i = i + 1;
waitfor(rateObj);
end

%%
figure;
title('Robot pose: x vs y');
plot(x_recorded, y_recorded);
figure;
%title('poseError');
plot(t_recorded, theta_recorded);
hold on;
plot(t_recorded, steeringDir, '*');
hold on;
plot(t_recorded, targetDir, '+');
hold on;
plot(t_recorded, v_recorded, '-');
hold on;
plot(t_recorded, omega_recorded, '-o-');
legend('theta_recorded','steeringDir','targetDir','v-recorded', 'omega-recorded');
hold off;