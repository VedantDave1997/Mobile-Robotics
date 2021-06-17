In this Section, we have introduced the Scan matching that basically provides an alternative to Monte
Carlo localization to compensate for the odometry drift.  The consecutive scans of the point cloud data is usually 
misaligned. This error is attributed to the uncertainty of the pose estimate and to possible
discrepancies between the time instance at which the laser scan is taken and the instance
at which the transform is computed. Thus initially we discuss about the Transformations in brief 
before going into detail.<br /><br />

<h3>Robotics System Toolbox Transformation Tree</h3>
The ROS transformation tree object provides access to the tf coordinate transformations
that are shared on the ROS network. These transformations canbe applied to objects such as
points, point clouds or poses.  ROS uses the tf transform library to keep
track of the relationship between multiple coordinate frames. The relative transformations
between these coordinate frames are maintained in a tree structure, which is queried while 
performing transformations.<br /><br />
Initially a tfTree is generated. Then we instantiate subscribers for the topics ’/move_base_simple/goal’ and
’/clicked_point’. The topic ’/clicked_point’ contains the most recent published
point in RVIZ. We transform the published point message from the topic ’/clicked_point’ into the
frames map, odom and base_link and convert the PointStamped message into its ordinary components in 
terms of double with function PointStampedMsg2Point.

<h3>Robotics System Toolbox Transformation Tree</h3>
We process the range data as a point cloud on the topic /laserpcl. The point cloud message
sensor_msgs/PointCloud2.msg contains the range readings as an array of 3D points with 
coordinates [xyz]. ROS has a dedicated Point Cloud Library (PCL) for point cloud processing. 
The PCL framework contains numerous state-of-the art algorithms including filtering, 
feature estimation, surface reconstruction, registration, model fitting, and segmentation.
We extract the coordinates from all points in the point cloud object, pcloud, and
returns them as an n-by-3 matrix of n 3-D point coordinates.

<h3>Scan Matching with the Normal Distribution Transform</h3>
The goal of scan matching is to find the transform between the two robot poses at which
the scans are taken. The scans are aligned based on the shapes of their overlapping features.
The NDT models the distribution of all reconstructed 2D-Points of one laser scan by a collection of local normal distributions.
First, the 2D space around the robot is subdivided regularly into cells with constant size.
Then for each cell, that contains at least three points, the following is done:<br />
- Collect all 2D-Points X<sub>i=1⋅⋅n</sub> contained in this box.<br />
- Calculate the mean q=∑x/n<br />
- Calculate the covariance matrix<br /><br />
The probability of measuring a sample at 2D-point x contained in this cell is now 
modeled by the normal distribution N(q,Σ):
<img src="https://render.githubusercontent.com/render/math?math=p(x) = \exp(\frac{-(x-q)^T\Sigma^{-1}(x-q)}{2})"><br />
Normal distribution transforms share some feature with probability occupancy grids. The occupancy 
grid represents the general probability of a cell being occupied, whereas the normal distribution
transform reflects the probability of a sample at a particular location x within the cell. Once the probability density is calculated, an optimization method estimates the relative pose between
the current laser scan and the reference laser scan. To speed up the convergence of the method,
an initial guess of the pose, typically obtained from odometry, is provided.<br /><br />

In order to implement the above mentioned algorithm, we instantiate a subscriber for the laser scan
topic and receive a reference scan message 'referenceScanMsg'. We Extract current and reference 
scan angles 'currScanAngles', 'refScanAngles' from the message with 'readScanAngles'. Within the 
while loop receive the current scan message 'currentScanMsg'. We determine the relative pose between 
the two scans. We then determine the true range readings that are below the maximum range reading, calculate
the homogeneous transform of the relative pose and integrate with the absolute transform by 
post multiplication of the two matrices. We then apply the absolute transform to the Cartesian 
coordinates of the scan points scanX, scanY that are in range. We plot the transformed scan points
superimposed on the previous plots and the current scan message becomes the reference scan message
of the next iteration of the loop.


