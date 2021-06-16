function [] = forward( velPub, distance, velocity )
time=distance/velocity;
rateObj=robotics.Rate(1/time);
rateObj.reset;
velMsg = rosmessage(velPub);
%[velPub, velMsg] = rospublisher('/mobile_base/commands/velocity');
velMsg.Linear.X=velocity;
i=0;
for i=1:time
    send(velPub,velMsg);
    waitfor(rateObj);
end

velMsg.Linear.X=0;
send(velPub,velMsg);
end

