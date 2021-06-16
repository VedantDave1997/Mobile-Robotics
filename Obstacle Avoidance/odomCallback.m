function [] = odomCallback(~,odomMsg,pose)
[pose.x, pose.y, pose.theta] = OdometryMsg2Pose( odomMsg );   
 

end