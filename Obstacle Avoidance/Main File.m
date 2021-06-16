clear all;
close all;
load('rangedata_case3.mat');

%%
steeringdirection = vfh(ranges, angles, targetdir, true);

%%
vfhPlus = robotics.VectorFieldHistogram;
vfhPlus.DistanceLimits = [0.05 5];
vfhPlus.RobotRadius = 0.1;
vfhPlus.MinTurningRadius = 0;
vfhPlus.SafetyDistance = 0.5;


%%
steeringAngle = vfhPlus(double(ranges), angles, targetdir);
