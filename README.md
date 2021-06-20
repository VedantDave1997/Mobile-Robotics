This repository consists of some basic methods and algorithms used in Mobile Robots field taught at 
Technische Universität Dortmund. In order to implement the algorithm on Robotic applications in 
simulation, Robot Operating System(ROS) is used. The  ROS environment is launched in Ubuntu 16.04. 
In particular, we use Stage and Gazebo simulator with the configuration of the Turtlebot robot in the
Stage simulator. We also utilise Rviz as the standard ROS tool for visualization of data, for example
scan data of a range finder. The Mathworks Robotics System Toolbox connects the ROS master and 
subscribes to ROS nodes.

<h3>ROS</h3>
ROS is an open-source, meta-operating system for your robot. It provides the common of an ordinary
operating system, including hardware abstraction, low-level device control, implementation of
commonly-used functionality, message-passing between processes, and package management. It also 
provides tools and libraries for obtaining, building, writing, and running code across multiple 
computers. ROS strongly supports reuse of software in robotics research and development. ROS is a 
distributed framework of processes (nodes) that enables executables to be individually designed and 
loosely coupled at runtime. We specifically ROS Kinetic for this purpose. The installation guidelines
about ROS Kinetic can be found at http://wiki.ros.org/ROS/Installation.

<h3>Stage and Gazebo</h3>
Stage is a standalone simulator with an interface to ROS. The integrated controllers enable motion 
control, robot behaviour and processes. Stage is a scaleable which allows the simulation of a fleet of
mobile robots residing in a two-dimensional bitmapped environment. Stage provides virtual robots 
which interact with simulated rather than physical devices. The Stage tutorials introduces the 2D
simulator and explains how to setup the Turtlebot robot and configure the robots environment. For 
further information see http://rtv.github.io/Stage/.<br/><br/>
Gazebo is a multi-robot simulator for indoor and outdoor environments. Like Stage it is able to
simulate multiple robots, sensors and objects. The main difference between Stage and Gazebo is that
the later resides in a three-dimensional world and supports the simulation of rigid-body physics,
namely robots that push things around, pick things up. Robots generally interact with the world in a
plausible manner, for example a Turtlebot robot starts to spin if centrifugal forces on a curved 
trajectory exceed a threshold.

<h3>Rviz</h3>
RViz uses the tf transform system for transforming data from the coordinate frame it arrives in, into
a global reference frame. There are two coordinate frames that are important to know about in the
visualizer. The fixed frame is the reference frame used to denote the world frame. This is usually 
the map, or world, or something similar, but can also be, for example, your odometry frame. For 
correct results, the fixed frame should not be moving relative to the world. The target frame is the 
reference frame for the camera view. For example, if your target frame is the map, you see the robot 
driving around the map from the perspective of an outside observer. Rviz provides a number of 
different camera perspectives so called views to visualize the environment. A display in rviz is 
something that draws something in the 3D world such as a point cloud or the robot state. The Rviz 
User Guide explains the GUI and the functionalities that Rviz provides. The Robot frames in RViz is 
shown in the figure below:<br/><br/>
<p align="center">
  <img src="Figures/Turtlebot in Rviz.JPG" width="350" title="hover text">
</p>
The modules covered includes:
<ul>
  <li>Scan Matching</li>
  <li>Reactive Obstacle Avoidance</li>
  <li>Robot Homing</li>
  <li>Obstacle Avoidance</li>
  <li>Monte Carlo Localization</li>  
</ul>