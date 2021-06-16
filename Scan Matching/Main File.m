%1
clear all;
tfTree = rostf;
tfTree.AvailableFrames;

%4
sub_goal= rossubscriber('/move_base_simple/goal');
sub_clicked_point=rossubscriber('/clicked_point');

%5
pt = rosmessage('geometry_msgs/PointStamped');
maxTime=20;
rateObj=robotics.Rate(10);


%6
i=1;
maxTime=20;
rateObj.reset;
while rateObj.TotalElapsedTime<maxTime
    pt_receive=receive(sub_clicked_point);
    ptmap=transform(tfTree,'map',pt_receive);
    ptbaselink = transform(tfTree,'base_link',pt_receive);
    ptodom=transform(tfTree,'odom',pt_receive);

    [xm(i), ym(i), zm(i) ]=PointStampedMsg2Point(ptodom);
    [xb(i), yb(i), zb(i) ]=PointStampedMsg2Point(ptbaselink);
    [xo(i), yo(i), zo(i) ]=PointStampedMsg2Point(ptodom);
    i=i+1;
end

subplot(2,2,1);
hold on;
plot3(xm,ym,zm,'o');
title('wrt map frame');

subplot(2,2,2);
hold on;
plot3(xo,yo,zo,'o');
title('wrt to odom frame')

subplot(2,2,3);
hold on;
plot3(xb,yb,zb,'o');
title('wrt to base frame');
hold on;
    plot(scanPoints(1,:),scanPoints(2,:));

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

% 10
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


%11
sub_laser=rossubscriber('/laserpcl');
laser_msg=receive(sub_laser);

canTransform(tfTree,'base_laser_link_0', 'map');
laser_map=transform(tfTree,'map',laser_msg);

xyz_pcloud=readXYZ(laser_map);
plot(xyz_pcloud(:,1),xyz_pcloud(:,2),'o')

i=1;
maxTime=20;
hold on;
plot(scanPoints(1,:),scanPoints(2,:));
figure;
rateObj.reset;
while rateObj.TotalElapsedTime<maxTime
    sub_laser=rossubscriber('/laserpcl');
    
    laser_msg=receive(sub_laser);
    %laser_map=transform(tfTree,'odom',laser_msg);
    laser_map=transform(tfTree,'map',laser_msg);
    xyz_pcloud=readXYZ(laser_map);
    hold on;
    plot(xyz_pcloud(:,1),xyz_pcloud(:,2),'o')
    
end

% 16 to 19
odomSubCallback = rossubscriber('/odom', @odomCallback);
clear odomSubCallback;

%19
%in odom accuracy is better.
ptcloud_map = transform(tfTree,'map',laser_msg,'msgtime');

% 20 to 22
pose = PoseHandle;
odomSubCallback =rossubscriber('/odom',{ @odomCallback, pose } );
clear odomSubCallback;

%21
sub_laser_scan=rossubscriber('/laser_scan');
referenceScanMsg=receive(sub_laser_scan);
refScanAngles=readScanAngles(referenceScanMsg);
currScanAngles=readScanAngles(referenceScanMsg);

i=1;
maxTime=40;
rateObj.reset;
prev_scan_msg=receive(sub_laser_scan);
prev_scan_msg_range=prev_scan_msg.Ranges;
prev_scan_angle=readScanAngles(prev_scan_msg);
while rateObj.TotalElapsedTime<maxTime
    %currentScanMsg=receive(sub_laser_scan);
    refScanRanges=prev_scan_msg_range;
    refScanAngles=prev_scan_angle;
    currentScanMsg=receive(sub_laser_scan);
    currScanRanges=currentScanMsg.Ranges;
    [relativepose, stats] = matchScans(currScanRanges, currScanAngles, refScanRanges,refScanAngles, 'SolverAlgorithm', 'fminunc');
    [scanX,scanY]=pol2cart(currScanAngles,currScanRanges);
    inRange=find(currScanRanges < double(currentScanMsg.RangeMax));
    absolutetform = eye(4);
    scanPoints=absolutetform*[scanX(inRange)';scanY(inRange)';zeros(1,length(inRange));ones(1,length(inRange))];
        
    relativetform = trvec2tform([relativepose(1) relativepose(2) 0])*eul2tform([relativepose(3) 0 0]);   %28
    
    absolutetform = absolutetform*relativetform;
    prev_scan_msg_range=currScanRanges;
    prev_scan_angle=currScanAngles;
    hold on;
  plot(scanPoints(1,:),scanPoints(2,:));
end

% 23
fig=figure(3);
pose = PoseHandle;
odomSubCallback =rossubscriber('/odom',{ @odomCallback, pose, fig } );
clear odomSubCallback;

 %30
    hold on;
    plot(scanPoints(1,:),scanPoints(2,:));
    
 %31
 map = robotics.OccupancyGrid(8, 8, 10);
 %32
  absolutepose_vec= tform2trvec(absolutetform);
  absolutepose_theta=tform2eul(absolutetform);
  absolute_pose=([absolutepose_vec(1),absolutepose_vec(2),absolutepose_theta(1)]);
  
 %33
 rateObj.reset;
while rateObj.TotalElapsedTime<maxTime
     sub_laser_scan=rossubscriber('/laser_scan');
     
    refScanRanges=prev_scan_msg_range;
    currentScanMsg=receive(sub_laser_scan);
    currScanRanges=currentScanMsg.Ranges;
    [relativepose, stats] = matchScans(currScanRanges, currScanAngles, refScanRanges,refScanAngles, 'SolverAlgorithm', 'fminunc');
    [scanX,scanY]=pol2cart(currScanAngles,currScanRanges);
    inRange=find(currScanRanges < double(currentScanMsg.RangeMax));
    absolutetform = eye(4);
    scanPoints=absolutetform*[scanX(inRange)';scanY(inRange)';zeros(1,length(inRange));ones(1,length(inRange))];
        
    relativetform = trvec2tform([relativepose(1) relativepose(2) 0])*eul2tform([relativepose(3) 0 0]);   %28
    
    absolutetform = absolutetform*relativetform;
    prev_scan_msg_range=currScanRanges;
    
    sub_laser_scan=rossubscriber('/laser_scan');
    referenceScanMsg=receive(sub_laser_scan);
    
     absolutepose_vec= tform2trvec(absolutetform);
  absolutepose_theta=tform2eul(absolutetform);
  absolute_pose=([absolutepose_vec(1),absolutepose_vec(2),absolutepose_theta(1)]);
    absolute_pos_pose=abs(absolute_pose);
     hold on;
    %plot(scanPoints(1,:),scanPoints(2,:),'o');
    plot(scanX,scanY,'o');
    insertRay(map,absolute_pos_pose,sub_laser_scan.LatestMessage.Ranges,readScanAngles(referenceScanMsg),double(sub_laser_scan.LatestMessage.RangeMax));
    %axes = show(map);
end
axes = show(map);
        


