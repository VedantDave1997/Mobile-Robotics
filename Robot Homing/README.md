In this Section, <br /><br />

<h3>Motion Control</h3>
Motion control of a mobile robot is concerned with tracking a feasible reference state
trajectory. The mobile robot regulates its planar pose via the control of its motors in
terms of velocity commands. The approaches rest upon feedback control of the actual
robots pose either w.r.t. a global fixed frame or w.r.t. the reference trajectory. A nonholonomic
robot possesses fewer local degrees of freedom than global degrees of freedom. The prototypical
 mobile robot is the unicycle with controls v, ω and global pose xr, yr, θr governed by the 
 kinematics

<h3>Callback Subscriber for Laser Scan Messages</h3>
We now focus with subscribing to the '/laserScan' topic and processing
the laser scan message with a Callback function.<br /> The 'sensor_msgs/LaserScan.msg' is
mapped onto a handle object of type ScanHandle which is composed of the properties
ranges and angles. Determine the distance and heading to the most imminent obstacle
with which the robot might collide if it does not alter its direction of motion. This obstacle
representation provides the basis for a reactive obstacle avoidance behavior.
The class ScanHandle has properties of '/laser_scan' topic i.e. Ranges(array of range readings in m) and 
Angles(array of angles in rad).<br />
A callback function for the '/laser_scan' topic converts the LaserScanMsg into to the ScanHandle object laserScan.<br /><br/>
The function 'nearestObstacle' determines the  distance r<sub>min</sub> and heading &phi;</sub><sub>min</sub> towards the most relevant
obstacle. The relevance of an obstacle depends on its proximity to the robot but
also to the relative heading. Nearby obstacles in longitudinal direction are far
more relevant for obstacle avoidance than remote obstacles or obstacles in lateral
direction.  Thus the scaled obstacle distance is given by<br />
<img src="https://render.githubusercontent.com/render/math?math=\hat{r}_i = (r_i - r_{robot})(1-\beta cos(\phi_i))">
The robot radius r<sub>robot</sub> is subtracted from the range readings r<sub>i</sub> as those measure the distance to the center of the robot
rather than the distance to the robots perimeter.

<h3>Reactive Obstacle Avoidance</h3>
This is a simple obstacle avoidance behavior which is in terms of a functional mapping between the laser range readings (Subscriber '/laser_scan') and the motor actions (Publisher '/mobile_base/commands/velocity'). Reutilize the handle class, the subscriber Callback and the function nearestObstacle from the previous assignments. The linear velocity v is reduced with decreasing range reading and eventually becomes zero at a stopping distance r<sub>stop</sub>. The robot is supposed
to stop if the scaled obstacle distance is below the threshold distance i.e. 
<img src="https://render.githubusercontent.com/render/math?math=\hat{r}_{min} = r_{min}(1-\beta cos(\phi_{min})) < r_{stop}">
The limitations are shown with the intuitive image below:
<p align="center">
  <img src="Figures/Reactive_obst.JPG" width="450" title="hover text">
</p>
The robot moves at maximum velocity v<sub>0</sub> if the obstacle distance exceeds a safe range r<sub>safe</sub>. In between r<sub>safe</sub> and r<sub>stop</sub> the robots translational velocity decreases linearly from v = v<sub>0</sub> to v = 0.
<p align="center">
  <img src="Figures/lin_vel_limit.JPG" width="450" title="hover text">
</p>
The fundamental relationship between scaled minimum range reading and turn rate &omega; is such that the magnitude of the turn rate increases with decreasing range reading. The robot is supposed to turn at maximum turn rate  &omega; =  &omega;<sub>max</sub> if the scaled obstacle distance r<sub>min</sub> is short of a distance r<sub>turn</sub>. The robot moves straight &omega; = 0 if the scaled obstacle distance exceeds a safe range r<sub>safe</sub>. In between r<sub>safe</sub> and r<sub>turn</sub> the robots turn rate increases linearly from &omega; = 0 to &omega; = &omega;<sub>max</sub>.
<p align="center">
  <img src="Figures/ang_vel_limit.JPG" width="450" title="hover text">
</p>
The sign of the turn rate depends on the obstacle direction &phi;<sub>min</sub> under which the nearest obstacle emerges w.r.t. the robocentric frame. The robot is supposed to turn away from the obstacle. Thus the sign of the turn rate &omega; is opposite to the sign of the obstacle direction &phi;<sub>min</sub>.
<img src="https://render.githubusercontent.com/render/math?math=\omega = sgn(\phi_{min})|\omega|">

These equations establish a purely reactive, memoryless behavior in which controls only depend on the current perception. It leads to robot getting stuck in corners by turning back and forth as&phi;<sub>min</sub> switches sign for the nearest obstacle to the left and right.  The current turning direction is maintained until the robot clears itself from the obstacle. The control law becomes:
<p align="center">
  <img src="Figures/rot_sign.JPG" width="350" title="hover text">
</p>

