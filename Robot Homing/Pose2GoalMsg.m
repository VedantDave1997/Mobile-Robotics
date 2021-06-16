function [ goalMsg ] = Pose2GoalMsg( x, y, theta )
% convert vector to pose stamped message
% 2017 - Frank Hoffmann
goalMsg=rosmessage('move_base_msgs/MoveBaseGoal');
goalMsg.TargetPose.Pose.Position.X=x;
goalMsg.TargetPose.Pose.Position.Y=y;
goalMsg.TargetPose.Pose.Position.Z=0;
q=eul2quat([theta, 0, 0]);
goalMsg.TargetPose.Pose.Orientation.W=q(1);
goalMsg.TargetPose.Pose.Orientation.X=q(2);
goalMsg.TargetPose.Pose.Orientation.Y=q(3);
goalMsg.TargetPose.Pose.Orientation.Z=q(4);
goalMsg.TargetPose.Header.FrameId='map';
end
