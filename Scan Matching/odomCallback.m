function [] = odomCallback( ~ , odomMsg)
global xr
global yr 
global thetar 
[ xr, yr, thetar ] = OdometryMsg2Pose( odomMsg );
quiver(xr,yr,thetar);

% figure(fig);
% [ pose.x, pose.y, pose.theta ] = OdometryMsg2Pose( odomMsg );
% quiver(pose.x,pose.y,pose.theta);
end