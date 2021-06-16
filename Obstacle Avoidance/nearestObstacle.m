function [ rmin, phimin ] = nearestObstacle( scanMsg )
  R =scanMsg.Ranges;
  Phi = readScanAngles(scanMsg);
  r_robot= 0.1;
  beta=0.8;
  r_h = (R - r_robot).*(1.0-beta*cos(Phi));
  [r_min, arg] = min(r_h);
  rmin=r_min;
  phimin = Phi(arg);
end