function [ x, y, z ] = PointStampedMsg2Point( pointMsg )
% convert odometry message to vector
x=pointMsg.Point.X;
y=pointMsg.Point.Y;
z=pointMsg.Point.Z;
end

