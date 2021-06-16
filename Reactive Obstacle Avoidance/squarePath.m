function[]=squarePath(actClient,sub)
x=sub.LatestMessage.Pose.Pose.Position.X;
y=sub.LatestMessage.Pose.Pose.Position.Y;
%theta=sub.LatestMessage.Pose.Pose.Orientation.Z;

goalMsg=Pose2GoalMsg( x+2, y+2, pi/2 );
sendGoalAndWait(actClient,goalMsg);
goalMsg=Pose2GoalMsg( x+2, y+4, pi/2 );
sendGoalAndWait(actClient,goalMsg);
goalMsg=Pose2GoalMsg( x+2, y+4, 0 );
sendGoalAndWait(actClient,goalMsg);
goalMsg=Pose2GoalMsg( x+4, y+4, 0 );
sendGoalAndWait(actClient,goalMsg);
goalMsg=Pose2GoalMsg( x+4, y+4, -pi/2 );
sendGoalAndWait(actClient,goalMsg);
goalMsg=Pose2GoalMsg( x+4, y+2, -pi/2 );
sendGoalAndWait(actClient,goalMsg);
goalMsg=Pose2GoalMsg( x+4, y+2, -pi );
sendGoalAndWait(actClient,goalMsg);
goalMsg=Pose2GoalMsg( x+2, y+2, -pi )
sendGoalAndWait(actClient,goalMsg);


end
