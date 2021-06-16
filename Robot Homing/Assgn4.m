clear all;
close all;
rosshutdown;
rosinit;
% roslaunch rst_mobile_robots turtlebot_in_stage.launch map_file:=empty move_base:=false amcl:=false
%% 1
global tftree_global;
global waypoints;
tftree_global = rostf();
odomSub =rossubscriber('/odom');
odomMsg=rosmessage(odomSub);
navGoalSub=rossubscriber('/move_base_simple/goal_odom');
velocityPub =rospublisher('/mobile_base/commands/velocity');
velocityMsg = rosmessage(velocityPub);
%%
clear x_recorded; clear y_recorded; clear rho_recorded; clear alpha_recorded; clear phi_recorded;
%% 2,3,4
maxTime=150;
rateObj=robotics.Rate(10);
rateObj.reset;
i = 1;
while rateObj.TotalElapsedTime<maxTime
navGoalMsg=navGoalSub.LatestMessage;
odomMsg=odomSub.LatestMessage;
[rho_recorded(i), alpha_recorded(i) , phi_recorded(i), deltay] = poseErrorTf( navGoalMsg, tftree_global );
[ v, omega] = homingControl( rho_recorded(i), alpha_recorded(i), phi_recorded(i) );

velocityMsg.Linear.X=v;
velocityMsg.Angular.Z=omega;
send(velocityPub,velocityMsg);
waitfor(rateObj);

[x_recorded(i), y_recorded(i), theta]=OdometryMsg2Pose(odomMsg);
t_recorded(i) = rateObj.TotalElapsedTime;
i = i + 1;
end

%% 5
figure;
title('Robot pose: x vs y');
plot(x_recorded, y_recorded, 'o');
figure;
title('poseError');
plot(t_recorded, rho_recorded, 'o');
hold on;
plot(t_recorded, alpha_recorded, '*');
hold on;
plot(t_recorded, phi_recorded, '+');
hold off;

%%  10
wayPointPub=rospublisher('/waypoint','geometry_msgs/PoseStamped');
navGoalSub=rossubscriber('/waypoint');
%%
waypoints = [5 0 0;0 0 pi/2;5/sqrt(2) 5/sqrt(2) 0;0 0 pi/2;0 5 0;0 0 pi/2;-5/sqrt(2) 5/sqrt(2) 0;0 0 pi/2;...
    -5 0 0;0 0 pi/2;-5/sqrt(2) -5/sqrt(2) 0;0 0 pi/2;0 -5 0;0 0 pi/2;5/sqrt(2) -5/sqrt(2) 0;0 0 pi/2];

odomWayPointSubCallback = rossubscriber('/odom',{@odomWayPointCallback, wayPointPub});


%% 11 &12
%clear x_recorded; clear y_recorded; clear rho_recorded; clear alpha_recorded; clear phi_recorded;
rateObj=robotics.Rate(20);
rateObj.reset;
i = 1;
while ~isempty(waypoints)
    
navGoalMsg=navGoalSub.LatestMessage;
odomMsg=odomSub.LatestMessage;
[rho_recorded(i), alpha_recorded(i) , phi_recorded(i), deltay] = poseErrorTf( navGoalMsg, tftree_global );
[ v, omega] = homingControl( rho_recorded(i), alpha_recorded(i), phi_recorded(i) );
velocityMsg.Linear.X=v;
velocityMsg.Angular.Z=omega;
send(velocityPub,velocityMsg);
waitfor(rateObj);

[x_recorded(i), y_recorded(i), theta]=OdometryMsg2Pose(odomMsg);
t_recorded(i) = rateObj.TotalElapsedTime;
i = i + 1;
end

%% 13
waypoints =  [4 4 0; 8 0 0; 12 4 0; 16 0 0];
lookAheadPointPub=rospublisher('/lookaheadpoint','geometry_msgs/PoseStamped');
navGoalSub=rossubscriber('/lookaheadpoint');
odomLookAheadPointSubCallback = rossubscriber('/odom',{@odomLookAheadPointCallback,lookAheadPointPub, 1});

%%
clear x_recorded; clear y_recorded; clear rho_recorded; clear alpha_recorded; clear phi_recorded;

%odomWayPointSubCallback = rossubscriber('/odom',{@odomWayPointCallback, wayPointPub});

%%
rateObj=robotics.Rate(20);
rateObj.reset;
i = 1;
while ~isempty(waypoints)
    
navGoalMsg=navGoalSub.LatestMessage;
odomMsg=odomSub.LatestMessage;
[rho_recorded(i), alpha_recorded(i) , phi_recorded(i), deltay] = poseErrorTf( navGoalMsg, tftree_global );
[ v, omega] = homingControlPurePursuit( rho_recorded(i), alpha_recorded(i), phi_recorded(i), deltay );
velocityMsg.Linear.X=v;
velocityMsg.Angular.Z=omega;
send(velocityPub,velocityMsg);
waitfor(rateObj);

[x_recorded(i), y_recorded(i), theta]=OdometryMsg2Pose(odomMsg);
t_recorded(i) = rateObj.TotalElapsedTime;
i = i + 1;
end
%% 
figure;
title('Robot pose: x vs y');
plot(x_recorded, y_recorded, 'o');
figure;
title('poseError');
plot(t_recorded, rho_recorded, 'o');
hold on;
plot(t_recorded, alpha_recorded, '*');
hold on;
plot(t_recorded, phi_recorded, '+');
hold off;