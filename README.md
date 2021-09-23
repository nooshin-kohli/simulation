# Gazebo simulation and ROS
## prerequisites
Considering ROS Melodic is installed on your Ubuntu 18.04, we will guide you to configure your workspace.
If you don't have ROS Melodic installed, check this page http://wiki.ros.org/melodic/Installation/Ubuntu
For using controllers you need to install ros_control and ros_controllers:
```
sudo apt update
```
```
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
```
For using joint-state-publisher-gui:
```
sudo apt update
```
```
sudo apt install ros-melodic-joint-state-publisher-gui
```
### create your workspace
At first, you need to configure a workspace using this commands:
```
mkdir <your_ws>
```
```
cd <your_ws>
```
```
catkin_make
```
For more information about catkin workspace check this page http://wiki.ros.org/catkin/Tutorials/create_a_workspace
Now It's time to clone the repository.
```
cd <your_ws>/src
```
```
git clone https://github.com/nooshin-kohli/simulation.git
```
There is no need to do all these steps all over again every time you want to use simulation. In the next part you will learn how to run this codes.
## Run simulation
Open new terminal
```
cd <your_ws>
source devel/setup.bash
```
To visualize robot with slider in rviz:
```
roslaunch first_leg urdf_rviz.launch
```
To visualize robot with slider in Gazebo:
```
roslaunch first_leg urdf_gazebo.launch
```
You can run rviz and gazebo simultaneously. To do so run gazebo as mentioned adove and in new terminal type:
```
rviz rviz
```
Now in rviz add tf plugin.
See Topics in new terminal write 
```
rostopic list
```
### Command a position to controllers
```
rostopic pub -1 /leg/jumper_position_controller/command std_msgs/Float64 <value in meters>
```
or for commanding one of revolute joints:
```
rostopic pub -1 /leg/hip_joint_position_controller/command std_msgs/Float64 <value in radians>
```
*note: for commanding negative values (for calf_joint) ```rostopic pub -1 /leg/calf_joint_position_controller/command std_msgs/Float64 -- <value in radians>``` 

## Run command node
After launching robot in gazebo, open another terminal then go to your workspace ```cd <your_ws>``` and source setup.bash ```source devel/setup.bash``` 
To run the node for no slider mode enter ```rosrun first_leg command.py``` 
you will see robot moving in gazebo.
*note: For permission denied error, open terminal in scripts directory and enter ```chmod +x command.py```
