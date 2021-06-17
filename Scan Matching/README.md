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