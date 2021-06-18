In this Section, <br /><br />

<h3>Motion Control</h3>
Motion control of a mobile robot is concerned with tracking a feasible reference state
trajectory. The mobile robot regulates its planar pose via the control of its motors in
terms of velocity commands. The approaches rest upon feedback control of the actual
robots pose either w.r.t. a global fixed frame or w.r.t. the reference trajectory. A non-holonomic
robot possesses fewer local degrees of freedom than global degrees of freedom. The prototypical
mobile robot is the unicycle with controls v, ω and global pose x<sub>r</sub>, y<sub>r</sub>, θ<sub>r</sub> governed by the 
kinematics:<br /><br />
<img src="https://render.githubusercontent.com/render/math?math={x'}_r =  vcos(\theta_r)">
<img src="https://render.githubusercontent.com/render/math?math={y'}_r =  vsin(\theta_r)">
<img src="https://render.githubusercontent.com/render/math?math={\theta'} =  \omega">
A differential drive robot is composed of two independent actuated wheels on a common
axle whose direction is rigidly connected to the robot body. The purpose of the passive orientable
caster wheel is to support the robot, it plays no role in motion control. The kinematic equations 
of the unicycle apply to the Turtlebot differential drive robot as well if one considers the 
robots frame origin to be located on the center of the virtual axle connecting the two driven wheels.
The differential drive robot kinematics are compliant with the unicycle model if left and right wheel
velocities [v<sub>l</sub>, v<sub>r</sub>] are mapped onto the vector of linear and angular velocity
[v, ω]. These are shown in the equations below:<br/><br/>
<img src="https://render.githubusercontent.com/render/math?math=v = \frac{v_r - (-v_l)}{2}">
<img src="https://render.githubusercontent.com/render/math?math=\omega = \frac{v_r - v_l}{b}">
in which b denotes the wheel base of the differential drive robot.



<h3>Feedback Robot Control with Publisher and Subscriber</h3>
The <i>/odom</i> topic provides the robot pose (x<sub>r</sub>, y<sub>r</sub>, θ<sub>r</sub>) w.r.t. 
the world frame X<sub>w</sub>, Y<sub>w</sub> . The <i>nav_msgs/Odometry</i> describes the 
orientation of the robot frame in terms of a quaternion q = [&eta; x y z]. As the mobile robot 
resides in a planar world, all rotations of the robot frame are along the z-axis only. The yaw angle 
θ<sub>r</sub> that denotes the rotation between the world x-axis X<sub>w</sub> and the robots x-axis
X<sub>r</sub> is extracted from the quaternion. The function <b>OdometryMsg2Pose</b> maps the 
message to a pose [x, y, theta].<br/><br/>

The code calculates the general pose error in robo-centric polar coordinates [ρ, α, φ] 
(x<sub>r</sub>, y<sub>r</sub>) and goal pose (x<sub>g</sub>, y<sub>g</sub>) and maps the error onto 
a motion command v, ω. ρ denotes the Euclidean distance between the current robot position and the 
goal position, α denotes the orientation error between the robot current heading in direction 
X<sub>r</sub> and the heading towards the goal pose (x<sub>g</sub>, y<sub>g</sub>). The angle φ
denotes the error between current robot heading and the desired orientation at the goal
pose. The angle φ becomes relevant later in the context of homing with goal poses
(x<sub>g</sub>, y<sub>g</sub>, θ<sub>g</sub>) rather than merely goal points (x<sub>g</sub>, y<sub>g</sub>).

<h3>Homing</h3>
Mobile robot homing is a simpler case of the general task of trajectory tracking as the goal pose is
static. One distinguishes between mere position based homing, in which the robot navigates towards a 
point x<sub>g</sub>, y<sub>g</sub> irrespective of the heading at which it approaches the target.
In homing with orientation the robot is expected to approach the target from a particular direction,
for example to enter a docking station for recharging. The robot is expected to acquire pose defined
in terms of x<sub>g</sub>, y<sub>g</sub>, θ<sub>g</sub>.<br/><br/>
Homing considers the current position and heading of the robot relative to a desired position and 
calculates a motion command based on their difference. The turn rate ω regulates the heading error
towards the goal point α to zero, the forward velocity is proportional to the Euclidean distance ρ
to the goal.

The fundamental relationship between scaled minimum range reading and turn rate &omega; is such that the magnitude of the turn rate increases with decreasing range reading. The robot is supposed to turn at maximum turn rate  &omega; =  &omega;<sub>max</sub> if the scaled obstacle distance r<sub>min</sub> is short of a distance r<sub>turn</sub>. The robot moves straight &omega; = 0 if the scaled obstacle distance exceeds a safe range r<sub>safe</sub>. In between r<sub>safe</sub> and r<sub>turn</sub> the robots turn rate increases linearly from &omega; = 0 to &omega; = &omega;<sub>max</sub>.
<p align="center">
  <img src="Figures/Robot Homing.JPG" width="450" title="hover text">
</p>
The sign of the turn rate depends on the obstacle direction &phi;<sub>min</sub> under which the nearest obstacle emerges w.r.t. the robocentric frame. The robot is supposed to turn away from the obstacle. Thus the sign of the turn rate &omega; is opposite to the sign of the obstacle direction &phi;<sub>min</sub>.
<img src="https://render.githubusercontent.com/render/math?math=\omega = sgn(\phi_{min})|\omega|">

These equations establish a purely reactive, memoryless behavior in which controls only depend on the current perception. It leads to robot getting stuck in corners by turning back and forth as&phi;<sub>min</sub> switches sign for the nearest obstacle to the left and right.  The current turning direction is maintained until the robot clears itself from the obstacle. The control law becomes:
<p align="center">
  <img src="Figures/rot_sign.JPG" width="350" title="hover text">
</p>