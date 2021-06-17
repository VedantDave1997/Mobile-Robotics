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
Then for each cell, that contains at least three points, the following is done:
- Collect all 2D-Points X<sub>i=1⋅⋅n</sub> contained in this box.
- Calculate the mean q=∑x/n
- Calculate the covariance matrix
