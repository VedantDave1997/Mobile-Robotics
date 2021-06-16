function groundTruthPose = ExampleHelperAMCLStageTruePose()
%ExampleHelperAMCLStageTruePose Returns the true robot pose from Stage.

% Obtain 'mobile_base' pose from stage.
posesub = rossubscriber('/base_pose_ground_truth');
stagePose = receive(posesub);
%.LatestMessage;
% Compute [x,y,yaw] from stage pose data
quat = [stagePose.Pose.Pose.Orientation.W, stagePose.Pose.Pose.Orientation.X, ...
    stagePose.Pose.Pose.Orientation.Y, stagePose.Pose.Pose.Orientation.Z];
rot = quat2eul(quat);
x = stagePose.Pose.Pose.Position.X;
y = stagePose.Pose.Pose.Position.Y;
yaw = rot(1);
groundTruthPose = [x,y,yaw];
end