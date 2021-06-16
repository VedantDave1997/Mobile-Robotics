function [] = odomLookAheadPointCallback( ~, odomMsg, wayPointSub , lookaheaddistance)
% Callback function for look ahead point on topic odom
% wayPointPub : publisher for waypoint navigation goals

persistent once

global waypoints tftree_global
global waypointcleared;
goalradius = 0.1;

if nargin < 4
    lookaheaddistance=1.0;
end


% Transform odom position into map frame
[x,y,t] = OdometryMsg2Pose(odomMsg);
odomPose = Pose2PoseStampedMsg(x, y,t);
odomPose.Header.FrameId = 'odom';
odomPoseMapMsg = transform(tftree_global,'map',odomPose);


if isempty(once) && ~isempty(waypoints)
    once=1;
    disp(['first waypoint ', mat2str(waypoints(1,:),2)]);
    disp(['first goalpoint ',mat2str(waypoints(2,:),2)]);
    [x, y, phi]=lookaheadpoint(odomPoseMapMsg.Pose.Position.X, odomPoseMapMsg.Pose.Position.Y, waypoints(:,1:2), lookaheaddistance);
    disp(['first lookAheadPoint ' , mat2str([x, y, phi],2)]);
end

if ~isempty(waypoints)
    
    % calculate look ahead point
    [x, y, phi, nextPathPointIdx]=lookaheadpoint(odomPoseMapMsg.Pose.Position.X, odomPoseMapMsg.Pose.Position.Y, waypoints(:,1:2), lookaheaddistance);
    
	% generate geometry_msgs/PoseTimeStamped Message in the map frame
    navGoalMsg=Pose2PoseStampedMsg(x, y, phi);
    navGoalMsg.Header.FrameId = 'map';
   	send(wayPointSub,navGoalMsg);

    % Disp current waypoint
    disp(['Next waypoint: ', mat2str(waypoints(nextPathPointIdx, :))]);
    
    % Check if final waypoint is reached
    if norm([odomPoseMapMsg.Pose.Position.X-waypoints(end,1), odomPoseMapMsg.Pose.Position.Y-waypoints(end,2)]) < goalradius && nextPathPointIdx >= size(waypoints, 1)
        disp(['final waypoint ', mat2str(waypoints(end,:),2), ' cleared.']);
        waypoints=[];
        waypointcleared=1;
    end
    
   

end

