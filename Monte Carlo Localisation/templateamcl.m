clear;
close all;
% Init ROS
rosshutdown;
rosinit('localhost');


%% Load map
mapimage=~logical(imread('/home/rosuser/Sow/Assignment6/rst_lab.png'));   % resolution 0.05 m/pixel
% entire RST lab environment
% map=robotics.BinaryOccupancyGrid(mapimage(1600:2600,1600:2600,1),20);
% map.GridLocationInWorld=[-20 -30];
% show(map);

% RST central lab 
map=robotics.BinaryOccupancyGrid(mapimage(1800:2200,1900:2300,1),20);
map.GridLocationInWorld=[-5 -10];

% figure(1);
% show(map);


%% Init AMCL

% init motion model and sensor model here

odometryModel = robotics.OdometryMotionModel;
rangefindermodel = robotics.LikelihoodFieldSensorModel;
rangefindermodel.Map=map;
rangeFinderModel.RandomMeasurementWeight = 1.0;
rangeFinderModel.ExpectedMeasurementWeight = 0.0;
odometryModel.Noise = [1.0 1.0 1.0 1.0];

% Query the Transformation Tree (tf tree) in ROS.
tftree = rostf;
% Get the transformation between the sensor and the robot base
waitForTransform(tftree,'/base_link','/camera_depth_frame');
sensorTransform = getTransform(tftree,'/base_link', '/camera_depth_frame');
rangeFinderModel.SensorPose = sensorTransform;

% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
    sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');


% Setup amcl
amcl = robotics.MonteCarloLocalization;
amcl.MotionModel=odometryModel;
amcl.SensorModel=rangefindermodel;
amcl.UseLidarScan = true;
amcl.ParticleLimits = [2000 10000];
amcl.GlobalLocalization = true;
%amcl.ParticleLimits = [200 2000];
%amcl.GlobalLocalization = false;
amcl.InitialPose = ExampleHelperAMCLStageTruePose(); % Get true initial pose
amcl.InitialCovariance = 0.2*eye(3);
visualizationHelper = ExampleHelperAMCLVisualization(map);

%% Create publisher and subscriber
% Create ROS subscribers for retrieving sensor and odometry measurements from TurtleBot.
laserSub = rossubscriber('scan');
odomSub = rossubscriber('odom');

% Create ROS publisher for sending out velocity commands to TurtleBot. TurtleBot subscribes to '/mobile_base/commands/velocity' for velocity commands.
[velPub,velMsg] = ...
    rospublisher('/mobile_base/commands/velocity','geometry_msgs/Twist');


%% Set up movement controller
% Robot motion is essential for the AMCL algorithm. 
% Drive TurtleBot randomly using the ExampleHelperAMCLWanderer class, which drives the robot inside the environment while avoiding obstacles using the robotics.VectorFieldHistogram class.
wanderHelper = ...
    AMCLWandererRST(sensorTransform, velPub, velMsg);
laserSub.NewMessageFcn = @wanderHelper.scan_callback;


%% Localization procedure
numUpdates=50;
trueposes=zeros(numUpdates,3);
odomposes=zeros(numUpdates,3);
amclposes=zeros(numUpdates,3);

i=1;
rate = 1; % Rate of 1 Hz
rateObj=robotics.Rate(rate); 
rateObj.reset;  % reset time at the beginning of the loop
while i <= numUpdates
    % Receive laser scan and odometry message.
    scanMsg = receive(laserSub);
    odompose = receive(odomSub);
    
    % Create lidarScan object to pass to the AMCL object.
    scan = lidarScan(scanMsg);
    
    % Compute robot's pose [x,y,yaw] from odometry message.
    [x, y, theta] = OdometryMsg2Pose(odompose);
    pose = [x y theta];
    
    % Perform localization here
    [isUpdated, estimatedPose, estimatedCovariance] = amcl(pose, scan);
    if isUpdated
        i = i + 1;
        plotStep(visualizationHelper, amcl, estimatedPose, scan, i);
    end
    trueposes(i,:)=ExampleHelperAMCLStageTruePose();
    odomposes(i,:)=pose;
    amclposes(i,:)=estimatedPose;
    [poses, weights]=amcl.getParticles();
    i = i + 1;
    waitfor(rateObj);
end
% Stop wanderer
wanderHelper.stop();
laserSub.NewMessageFcn = [];
load('particles.mat');
figure(3);
odomposes(:,1)=odomposes(:,1)+2.0; % offset odom map
odomposes(:,2)=odomposes(:,2)+2.0; % offset odom map
plot(trueposes(:,1), trueposes(:,2), 'bo', amclposes(:,1), amclposes(:,2), 'r*', odomposes(:,1), odomposes(:,2), 'g+'); 
title('estimated, true and odom robot poses');
legend('true pose','AMCL pose','odom pose');
plot3(particles(:,:,1),particles(:,:,2),particles(:,:,3),'b.');
hold on;

%----22----
amclstep=size(particles,1);
data=reshape(particles(amclstep,1:nparticles(amclstep),:),...
nparticles(amclstep),3);
kdtree=KDTreeSearcher(data);
point=[2 2 0];
[neighbour,D]=knnsearch(kdtree,point,'K',500);
for k=1:500
    xdata(k,1)=kdtree.X(neighbour(1,k),1);
    ydata(k,1)=kdtree.X(neighbour(1,k),2);
    zdata(k,1)=kdtree.X(neighbour(1,k),3);
   
end
%plot3(data(:,1),data(:,2),data(:,3),'b.');
%hold on;
plot3(xdata,ydata,zdata,'r.');
%-----24-25----
% pd = makedist('Normal');
% cdf_normal = cdf(pd,xdata);
% plot(x,cdf_normal)
x1 = -5:1:15;
y1 = -10:1:10;
theta = -pi:pi/4:pi;
[X,Y,Theta] = meshgrid(x1,y1,theta);
V=zeros(size(X));
for i=1:numel(X)
V(i)= 
end
rosshutdown;
