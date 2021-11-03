## Content
The repository contains a set of packages and resources for the ROS simulation of an array of two robotic arms.

The basic objective is the composition and testing of an algorithm for the planning of trajectories that allows to avoid collisions between the robots involved.

ROS packages:
<ol>
  <li>two_arms: system initialization package</li>
  <li>two_arms_description: description of the robotic system</li>
  <li>two_arms_moveit: description in MoveIt of the system</li>
</ol>

Upgrade Moveit:
<ol>
  <li>moveit_upgrade: contains the updated / corrected version of MoveIt's python interface</li>
</ol>

## How-To

### Upgrade Moveit
``sudo ./moveit_upgrade/upgrade.sh``

Script for installing the updated MoveIt python interface. Remember to run the script after elevating the privileges.

If not already done: ``chmod +x ./moveit_upgrade/upgrade.sh``

### Gazebo + MoveIt
``roslaunch two_arms bringup.launch``

Command for launching the entire system (Gazebo + Moveit + Rviz), excluding the Python script.

Alternatively, they can be started separately as follows.
### Gazebo
The system foresees the use of Gazebo for the simulation.

``roslaunch two_arms gazebo.launch``

### MoveIt Rviz
MoveIt and Rviz are required for planning.

``roslaunch two_arms planning_execution.launch``

### Commander python node
Take advantage of MoveIt's python interface for trajectory control and planning.

``roslaunch two_arms py_node``

### Testing

``rosrun two_arms test_suit.py``

### Benchmarks

``roslaunch two_arms py_node.launch package:=two_arms_benchmarks script:=algorithms_benchmarks.py``

