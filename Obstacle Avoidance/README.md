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
and bins and edges are calculated from the angles with <i>histcounts</i>. The histogram generated from the
data is shown below:
<p align="center">
  <img src="Figures/Histogram VFH.JPG" width="350" title="hover text">
</p>








