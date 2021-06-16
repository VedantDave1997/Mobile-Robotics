function [ x, y, theta ] = OdometryMsg2Pose( poseMsg )
% convert odometry message to vector
% 2007 - Frank Hoffmann
x=poseMsg.Pose.Pose.Position.X;
y=poseMsg.Pose.Pose.Position.Y;
eul=quat2eul([poseMsg.Pose.Pose.Orientation.W poseMsg.Pose.Pose.Orientation.X  poseMsg.Pose.Pose.Orientation.Y poseMsg.Pose.Pose.Orientation.Z]);
theta=eul(1);
end

