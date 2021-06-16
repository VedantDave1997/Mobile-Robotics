function [] = scanCallback1(scanMsg, velPub, navGoalSub, tftree, poseR)

komega = 2.0; % gain for turn rate
omegamin = -1.5;
omegamax = 1.5;
vsafe = 0.5;
goalRadius = 0.5;
rsafe = 0.5;
rstop=0.2;

global steeringDir v_recorded omega_recorded targetDir i;

%scanSub = rossubscriber('/scan');
%scanMsg = scanSub.LatestMessage;

velMsg=rosmessage(velPub);
navGoalMsgRobot = transform(tftree,'base_link', navGoalSub.LatestMessage);
[poseR.x, poseR.y, poseR.theta] = PoseStampedMsg2Pose(navGoalMsgRobot);
[targetdir, rho] = cart2pol(poseR.x, poseR.y);
angles = readScanAngles(scanMsg);
ranges = scanMsg.Ranges;
steeringAngle = vfh(ranges, angles, targetdir ,false);
[rmin, phimin] = nearestObstacle(scanMsg);

if(rmin<rstop)
    v= 0;
elseif(rmin>= rstop & rmin<=rsafe)
    v= vsafe*(rmin-rstop)/(rsafe-rstop);
elseif(rmin>rsafe)
    v=vsafe;
end

if (isnan(steeringAngle) & targetdir >= 0)
    omega = omegamax;
elseif (isnan(steeringAngle) & targetdir < 0)
    omega = omegamin;
elseif(~isnan(steeringAngle))
    S = saturation('LinearInterval',[omegamin,omegamax]);
    omega = evaluate(S,komega*steeringAngle);
end

if(rho <= goalRadius)
    v = 0;
    omega = 0;
end


steeringDir(i) = steeringAngle;
targetDir(i) = targetdir;
v_recorded(i) = v;
omega_recorded(i) = omega;
velMsg.Linear.X = v;
velMsg.Angular.Z = omega;
send(velPub,velMsg);
% 
% velMsg.Linear.X = 0;
% velMsg.Angular.Z = 0;
% send(velPub,velMsg);

end

