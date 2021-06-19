In this Section, we initially discuss controlling the robot velocity and path simply through the velocity
command. Then the robot is subjected to the constraint o<br /><br />

<h3>Motion Planning and Obstacle Avoidance</h3>
Motion planning is concerned with the computation of a collision-free trajectory to the target 
configuration that complies with the vehicle constraints. These techniques rely upon an accurate map
of the environment. and solve the navigation problem in a complete and global manner, in other words,
they find a collision free path to the goal if it exists. Many architectures including the ROS 
navigation stack combine the global path planning with a local obstacle avoidance scheme. The global 
path provides intermediate way-points that constitute temporal targets for the local sensor based 
obstacle avoidance.<br /><br />
The obstacle avoidance problem consists of computing a robot motion that avoids a
collision with the local obstacles detected by the range sensors. At the same time, the control is
supposed to bring the robot closer to the goal pose. This type of feedback control generates a 
sequence of motions that steer the robot towards the target on a collision-free path. Let 
q<sub>target</sub> denote a target pose and q<sub>t<sub>i</sub></sub> the current pose. 
In the case of a mobile robot moving in a plane, the pose is defined by the vector
qti = (x<sub>t<sub>i</sub></sub>, y<sub>t<sub>i</sub></sub>, θ<sub>t<sub>i</sub></sub>)<br /><br />.
At time t<sub>i</sub> the robot A is at q<sub>t<sub>i</sub></sub> and perceives the local obstacles 
O(q<sub>t<sub>i</sub></sub>) by means of a range reading S(q<sub>t<sub>i</sub></sub>). The objective
is to compute a motion command u<sub>i</sub> such that:
<ul>
  <li>the generated trajectory from pose q<sub>t<sub>i</sub></sub> towards the new pose 
  q<sub>t<sub>i+1</sub></sub> is collision free</li>
  <li>and the new pose is closer to the target F(q<sub>t<sub>i</sub></sub>,q<sub>target</sub>)<
  F(q<sub>t<sub>i</sub>+T</sub>,q<sub>target</sub>)
  </li>
</ul>
in which the scalar function F measures the progress of the current pose towards a target
pose. The drawback of the risk that the robot gets trapped due to the lack of global planning is 
compensated by the advantage of incorporating sensor information into the motion control problem.

<h3>Vector Field Histogram Method</h3>
The methods of subset of controls first determine a candidate set of feasible motion controls. 
The optimal control is selected from the candidate set according to some optimality criterion, 
usually related to the target heading. Vector field histogram first computes a set of obstacle-free 
candidate directions and then selects the direction closest to the target heading. It calculates the
steering direction based on the laser scan data and then converts the steering direction to an 
appropriate linear and an angular velocity (v, ω). If there is no feasible steering direction the 
robot stops and scans for a free direction by rotating in place. The Steering direction is determined
with the help of Laser Data through <b>steeringdirection</b> function . The input parameters 
<i>ranges</i> and <i>angles</i> denote the laser range data extracted from the scan message. 
The parameter <i>targetdir</i> denotes the heading towards the target in robocentric coordinates 
obtained from transforming the navigation goal message from the map frame to the base link frame. 
The output parameter <i>steeringdirection</i> is the obstacle-free direction θ<sub>k<sub>sol</sub></sub> 
in robocentric coordinates based on the vector field histogram algorithm. Angles are specified in
radians w.r.t. the robots current heading. The figure below shows the obstacles present in the map
and the directions that the robot is deciding upon.<br/><br/>
<p align="center">
  <img src="Figures/Obstacle in VFH.JPG" width="350" title="hover text">
</p>

A polar histogram is constructed in which each bin represents the obstacle polar density in the
corresponding sector. For that purpose, those laser scan readings r<sub>i</sub>, θ<sub>i</sub> that
belong to the k-th sector Ω<sub>k</sub> = {θ<sub>i</sub> ∈ [θ<sub>k</sub>,θ<sub>k+1</sub>]} are 
weighted by the inverse distance to the robot center r<sub>i</sub> and are accumulated according to:<br/><br/>
<img src="https://render.githubusercontent.com/render/math?math=h_k =  \Sigma_i(1-\frac{r_i}{r_{max}})^\alpha"><br/>

Due to the discrete nature of the histogram grid, the mapping from range readings to polar
histogram may appear ragged and cause errors in the selection of the steering direction.
Therefore, the histogram is smoothed with a spatial low pass filter according to:<br/><br/>
<img src="https://render.githubusercontent.com/render/math?math=h'_k =  \Sigma_{-l}^{l}(l-|i| %2B 1)h_{k %2B i}"><br/><br/>
The range data is converted into a polar obstacle density histogram from the laser range reading. 
We determine the <i>sectorincrement</i> as the angular separation of sectors and generate the array 
of <i>sectormidpoints</i> with linspace at equally spaced angular interval and the 
<i>sectorstartpoints</i> as the lower bound of the sector intervals. The range data is filtered by 
selecting only the range data with r<sub>i</sub> < r<sub>max</sub>. The vector of <i>weightedranges</i>
is calculated from from the <i>validranges</i>, <i>rmax</i> and <i>alpha</i>. A histogram is generated
and bins and edges are calculated from the angles with <i>histcounts</i>. The set of candidate directions
denoted by the term candidate valley is determined as the set of adjacent sectors S for the histogram
components below a threshold. The logical array of occupied sectors occupiedsectors (non candidate 
valleys) with a polar density above the threshold  h > h<sub>max</sub> is determined. The histogram 
generated from the data is shown below:
<p align="center">
  <img src="Figures/Histogram VFH.JPG" width="350" title="hover text">
</p>
The optimal direction is selected from the candidate valley according to the goal sector
and the following heuristics. Four distinctive cases are investigated in sequence:
<ul>
  <li>The goal sector k<sub>target</sub> is within the selected valley. Solution: k<sub>sol</sub> 
  = k<sub>target</sub> , where k<sub>target</sub> is the sector that contains the goal location.</li>
  <li> The goal sector is not in the selected valley and the number of sectors that form the candidate
   valley is greater than m. Solution: k<sub>sol</sub> = k<sub>i</sub> ± m/2, where m is a fixed number
   of sectors and k<sub>i</sub> the sector of the valley closer to the target sector k<sub>target</sub>.</li>
  <li>The goal sector is not in the selected valley and the number of sectors of the valley is lower or
   equal to m. Solution: k<sub>sol</sub> = (k<sub>i</sub> + k<sub>j</sub>)/2 is the bisector of
   k<sub>i</sub> and k<sub>j</sub> j which denote the extreme sectors of the valley. </li>
  <li>The candidate valley S = &Phi; is empty. The robot rotates in place and scans its environment
   until an obstacle free direction is perceived. </li>
</ul>

After the VFH generation, we implement a reactive obstacle avoidance behavior based on the vector
field histogram and determine an appropriate turn rate ω that turns towards the desired steering
direction and a safe linear velocity. Let h'<sub>c</sub> denote the smoothed polar obstacle density 
in the current direction of travel. Large values of h'<sub>c</sub> indicate that either a large 
obstacle lies ahead or a smaller obstacle is close to the robot. Either case demands a sharp turn of
the robot. This is achieved by a reduction of the linear velocity in order to allow completion of the 
turn towards the novel direction. The linear velocity is determined according to:<br/><br/>
<img src="https://render.githubusercontent.com/render/math?math=v =  v_0 max(0,(1-h'_c/h_m))"><br/><br/>
and h<sub>m</sub> is an empirical parameter that achieves the desired reduction in speed.
The above control law reduces the linear velocity of the robot in anticipation of an steering
manœuver. We then consider that steering directions that deviate from the current robots heading 
require an ongoing turning. Thus the linear velocity v is reduced to v<sub>red</sub> in proportion to 
the turn rate:<br/><br/>
<img src="https://render.githubusercontent.com/render/math?math=v_{red} =  v(1-|\omega|/\omega_{max}) %2B v_{min} "><br/><br/>
in which v denotes the linear velocity, ω<sub>max</sub> denotes the maximum admissible turn rate and
v<sub>min</sub> is a small non-zero velocity offset.<br/><br/>
The steering direction k<sub>sol</sub> is mapped onto the turn rate:<br/><br/>
<img src="https://render.githubusercontent.com/render/math?math=\omega=  sat(k_\omega \theta_{k_{sol}}) "><br/><br/>
in which θ<sub>k<sub>sol</sub></sub> denotes the direction of the bisector of the solution sector
k<sub>sol</sub> in the robo-centric frame. In case 4 the robot stops and rotates in place v = 0, 
ω = ω<sub>max</sub>.<br/>

The goal pose in robo-centric coordinates is obtained by mapping the navigation goal pose 
(topic <i>/move_base_simple/goal</i>) into the robo-centric frame base_link using the appropriate 
transform. The motion commands are published on the topic <i>/mobile_base/commands/velocity</i>.
The callback function <b>scanCallback</b> determines the steering direction from the scan message 
by means of VFH. The steering directionn and obstacle distance is mapped onto the corresponding 
motion command in terms of velocity and turn rate. The velocity command is published on the topic 
<i>/mobile_base/commands/velocity</i>. The callback function obtains a reference to a <i>PoseHandle</i> 
object by which it reports the navigation goal pose in the robo-centric frame back to the main program.
The main loop is governed by a rate object merely monitors progress towards the goal and terminates 
once the navigation goal is reached.