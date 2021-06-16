rosshutdown
clc;
clear all;
close all;

rosinit 'localhost'
rostopic list
rostopic info /scan
rostopic info /odom
rostopic info /mobile_base/commands/velocity

%7
scanSub = rossubscriber('/scan');
laserMsg = receive(scanSub,10);
% lasermsg = scanSub.LatestMessage;
fig1 = figure;
fig1=plot(laserMsg,'MaximumRange',7);

%8-9
rate = 5; % Loop Rate in Hz number of iterations / sec
maxTime = 40; % Maximum loop time (loop terminate after maxTime sec)
rateObj=robotics.Rate(rate);
rateObj.reset; % reset time at the beginning of the loop
fig2=figure;
while rateObj.TotalElapsedTime < maxTime
    % Receive laser scan message and plot range data
    laserMsg = scanSub.LatestMessage;
    waitfor(rateObj);
    fig2=plot(laserMsg,'MaximumRange',40);
%     hold on;
end
% hold off;


% 10 to 15
odomSub = rossubscriber('/odom');

rate = 5; % Loop Rate in Hz number of iterations / sec
maxTime = 40; % Maximum loop time (loop terminate after maxTime sec)
rateObj=robotics.Rate(rate);
rateObj.reset; % reset time at the beginning of the loop
fig3=figure;
while rateObj.TotalElapsedTime < maxTime
    % Receive laser scan message and plot range data
    odomMsg = receive(odomSub);
    pose = odomMsg.Pose.Pose;
    x = pose.Position.X;
    y = pose.Position.Y;
    waitfor(rateObj);
    fig3=plot(x,y,'*');
    hold on;
end
hold off;

 
odomCallback( '/odom' , odomMsg)


% 16 to 19
odomSubCallback = rossubscriber('/odom', @odomCallback);
clear odomSubCallback;

% 20 to 22
pose = PoseHandle;
odomSubCallback =rossubscriber('/odom',{ @odomCallback, pose } );
clear odomSubCallback;

% 23
fig=figure(3);
pose = PoseHandle;
odomSubCallback =rossubscriber('/odom',{ @odomCallback, pose, fig } );
clear odomSubCallback;