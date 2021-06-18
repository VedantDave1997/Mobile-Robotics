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
to the goal. The image below shows the Initial and Goal pose of the mobile robot.
<p align="center">
  <img src="Figures/Robot Homing.JPG" width="450" title="hover text">
</p>
We calculates the pose error in polar coordinates ρ, α from the pose messages <i>navGoalMsg</i>. 
Then we determine the goal pose in robo-centric coordinates by transforming the entity <i>navGoalMsg</i>
specified w.r.t. map frame to the ’base_link’ with the function transform. (see lab Robotics System Toolbox III).
Then we obtain the resulting goal pose x<sup>(r)</sup><sub>g</sub>, y<sup>(r)</sup><sub>g</sub>, 
θ<sup>(r)</sup><sub>g</sub> with the function <i>PoseStampedMsg2Pose</i>. The goal position error 
becomes:<br/><br/>
<img src="https://render.githubusercontent.com/render/math?math=\rho = \sqrt{{x^{(r)}}^2_g %2B {y^{(r)}}^2_g}">
<img src="https://render.githubusercontent.com/render/math?math=\alpha = \atan2(y^{(r)}_g,x^{(r)}_g)">
We then implement a simple proportional controller to regulate the heading error α and
the translational error ρ to zero.<br/><br/>
<img src="https://render.githubusercontent.com/render/math?math=v = \kappa_\rho \rho">
<img src="https://render.githubusercontent.com/render/math?math=\omega = \kappa_\alpha \alpha">
The plot of the evolution of the pose error [ρ, α] over time is shown below.
<p align="center">
  <img src="Figures/Error_vs_Params.JPG" width="450" title="hover text">
</p>

<h3>Trajectory Tracking and Homing with Goal Pose</h3>
Perfect tracking is only achieved if the reference trajectory is feasible for the robot and compliant
with the mobile robot kinematics. The turning radius of a car like robot is bounded from below, which
imposes an upper limit on the reference path curvature. Kinodynamic constraints impose bounds on the 
forward velocity v<sub>max</sub>and turn rate ω ∈ [ω<sub>min</sub>, ω<sub>max</sub>]. In general case
the robots orientation θ(t) is supposed to follow a reference orientation θ<sub>g</sub>(t) as well. 
For the unicycle robot the reference orientation is tangential to the path in case of a feasible 
reference trajectory. A general reference trajectory (x<sub>g</sub>(t), y<sub>g</sub>(t), θ<sub>g</sub>(t)) 
is feasible if it is generated by a reference vehicle with the same kinematics. With the new coordinates e = [ρ, α, φ] the
kinematics are described by:<br/><br/>
<img src="https://render.githubusercontent.com/render/math?math=\rho' = -vcos\alpha">
<img src="https://render.githubusercontent.com/render/math?math=\alpha' = \frac{vsin\alpha}{\rho}-\omega">
<img src="https://render.githubusercontent.com/render/math?math=\phi' = -\omega">
The transformation applies for α ∈ I<sub>1</sub> = (−π/2, π/2]. In case α ∈ I<sub>2</sub> = (−π, −π/2] ∪
(π/2, π] redefining the forward direction of the robot, by setting v = −v, and applying a similar transformation yields:<br/><br/>
<img src="https://render.githubusercontent.com/render/math?math=\rho' = vcos\alpha">
<img src="https://render.githubusercontent.com/render/math?math=\alpha' = \frac{-vsin\alpha}{\rho}-\omega">
<img src="https://render.githubusercontent.com/render/math?math=\phi' = -\omega">
with again α(0) ∈ I<sub>1</sub>. If α ∈ I<sub>1</sub>, the forward direction of the robot points toward 
the goal, whereas if α ∈ I<sub>2</sub>, the backward direction of the robot points toward the goal. 
In the first case the robot moves with a positive linear velocity v, whereas in the second case the
translational motion is reversed.<br/><br/>
<img src="https://render.githubusercontent.com/render/math?math=v = \kappa_\rho \rho">
<img src="https://render.githubusercontent.com/render/math?math=v = \kappa_\alpha \alpha %2B \kappa_\phi \phi">
The linear control law achieves an exponential stabilization of the system at the equilibrium 
(ρ, α, φ) = (0, 0, 0) under the following conditions on the gain:
<img src="https://render.githubusercontent.com/render/math?math=\kappa_\rho > 0">
<img src="https://render.githubusercontent.com/render/math?math=\kappa_\phi < 0">
<img src="https://render.githubusercontent.com/render/math?math=\kappa_\alpha %2B \kappa_\phi - \kappa_\rho > 0">
The image of the Global frame, robo-centric frame and goal frame for homing with exponential
stabilization is shown in the figure below.
<p align="center">
  <img src="Figures/Robot Homing Goal.JPG" width="450" title="hover text">
</p>
The resulting path generated due to the Goal pose constraint is shown in the figure below.
<p align="center">
  <img src="Figures/Robot Path Results.JPG" width="450" title="hover text">
</p>

<h3>Pure Pursuit Tracking Algorithm</h3>
The pure pursuit algorithm has the objective to guide a robot along a reference path. Pure pursuit 
is a path tracking scheme that determines the curvature of the robot path that guides the robot from 
its current pose to the goal pose. For that purpose the scheme considers a dynamic goal position on 
the path located some distance ahead of the robots current position. The robot is supposed to chase 
the moving goal point (look ahead point) on the path. This strategy is similar to human drivers that 
steer a vehicle towards a dynamic lookahead point on the road, which distance depends on the vehicle
speed, road curvature and visibility.<br/><br/>
In compliance with ROS transforms the x-axis of coordinate frame coincides with robots current 
heading and the y-axis with the axle connecting the two wheels. The vector (x<sub>r</sub> − x, y<sub>r</sub> − y) 
denotes the location of the lookahead (goal) point in the robocentric frame, l denotes the lookahead 
distance. In the following we consider all vectors to be expressed w.r.t. robot base frame. The 
algorithm calculates the radius of the arc that connects the origin of the robocentric frame with 
the goal point. The following relationships hold:<br/><br/>
<img src="https://render.githubusercontent.com/render/math?math=\rho^2 = (x_g-x_r)^2 %2B  (y_g-y_r)^2 = {x^{(r)}}^2_g %2B  {y^{(r)}}^2_g">
<img src="https://render.githubusercontent.com/render/math?math=r = y_g - y_r %2B d = y^{(r)}_g %2B d">
the first one is the distance to the lookahead point, the second relates the radius of the arc r and
the lateral offset of the robot y<sub>g</sub> − y<sub>r</sub> from the goal point. Combining the 
equations and solving for the searched radius r yields:
<img src="https://render.githubusercontent.com/render/math?math=r = \frac{\rho^2}{2y^{(r)}_g}">
The following figure shows the Geometry of the Pure Pursuit Algorithm:
<p align="center">
  <img src="Figures/Pure Pursuit Algorithm.JPG" width="450" title="hover text">
</p>
The remaining issue is to determine the lateral offset of the robot w.r.t. to the reference path.
The implementation involves the following step:
<ul>
  <li>Find the point (x<sub>p</sub>, y<sub>p</sub>) on the path closest to the current robot position</li>
  <li>Determine the lookahead point (x<sub>r</sub>, y<sub>r</sub>) on the path</li>
  <li>Transform the lookahead point into robocentric coordinates</li>
  <li>Select a constant velocity v and compute turn rate ω from curvature γ</li>
</ul>
The following figure shows the Look ahead point for a path composed of straight line segments with 
look ahead point on the next segment:
<p align="center">
  <img src="Figures/Look Ahead Path.JPG" width="450" title="hover text">
</p>