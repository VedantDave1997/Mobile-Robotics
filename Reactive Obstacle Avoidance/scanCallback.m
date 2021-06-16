function [] = scanCallback( ~,LaserScanMsg, laserScan)

angle=readScanAngles(LaserScanMsg);

laserScan.Angles_rad=angle;

laserScan.Ranges_m=LaserScanMsg.Ranges;


end
