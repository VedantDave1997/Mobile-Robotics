function [] = odomWayPointCallback( ~, odomMsg, wayPointSub )
% Callback function for waypoint on topic odom
% wayPointPub : publisher for waypoint navigation goals

persistent once

global waypoints tftree_global
goalradius = 0.1;
goalangle = deg2rad(3);


if isempty(once) && ~isempty(waypoints)
    once=1;
    disp(['first waypoint ', mat2str(waypoints(1,:),2)]);
end

if ~isempty(waypoints)
    
    % Transform odom position into map frame
    [x,y,t] = OdometryMsg2Pose(odomMsg);
    odomPose = Pose2PoseStampedMsg(x, y,t);
    odomPose.Header.FrameId = 'odom';
    odomPoseMapMsg = transform(tftree_global,'map',odomPose);
    
    
    % generate geometry_msgs/PoseTimeStamped Message
    % Publish goal in map frame
    navGoalMsg=Pose2PoseStampedMsg(waypoints(1,1), waypoints(1,2), waypoints(1,3));
    navGoalMsg.Header.FrameId = 'map';
    send(wayPointSub,navGoalMsg);
    
    
    [x, y, t] = PoseStampedMsg2Pose(odomPoseMapMsg);
    if norm([x-waypoints(1,1), y-waypoints(1,2)]) < goalradius && abs(angdiff(waypoints(1,3), t)) < goalangle
        if size(waypoints,1)>1
            disp(['waypoint ', mat2str(waypoints(1,:),2), ' cleared.']);
            disp(['next waypoint ', mat2str(waypoints(2,:),2)]);
        else
            disp(['final waypoint ', mat2str(waypoints(1,:),2), ' cleared.']);
        end
        waypoints=waypoints(2:end,:);
    end
end

end

