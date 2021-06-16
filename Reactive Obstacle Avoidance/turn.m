function [] = turn( velPub, angle, turnrate )
time=angle/turnrate;
rateObj=robotics.Rate(1/time);
rateObj.reset;
velMsg=rosmessage(velPub);
velMsg.Angular.Z=angle;
for i=1:time
    send(velPub,velMsg);
    waitfor(rateObj);
end
velMsg.Angular.Z=0;
send(velPub,velMsg);
end