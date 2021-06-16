function [ poseMsg ] = Pose2PoseStampedMsg( x, y, theta )
% convert vector to pose stamped message 
poseMsg=rosmessage('geometry_msgs/PoseStamped');
poseMsg.Pose.Position.X=x;
poseMsg.Pose.Position.Y=y;
poseMsg.Pose.Position.Z=0;
q=eul2quat([theta, 0, 0]);
poseMsg.Pose.Orientation.W=q(1);
poseMsg.Pose.Orientation.X=q(2);
poseMsg.Pose.Orientation.Y=q(3);
poseMsg.Pose.Orientation.Z=q(4);
end

