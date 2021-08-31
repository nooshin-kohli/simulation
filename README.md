# Gazebo simulation and ROS
## prerequisites
<p>Considering ROS Melodic is installed on your Ubuntu 18.04, we will guide you to configure your workspace.
If you dont have ROS Melodic installed check this page [http://wiki.ros.org/melodic/Installation/Ubuntu](url)</p>
### create your workspace
<p>At first you need to configure a workspace using this commands:</p>
`mkdir <your_ws>`<br>
`cd <your_ws>`<br>
`catkin_make`<br>
<p>For more information about catkin workspace check this page [http://wiki.ros.org/catkin/Tutorials/create_a_workspace](url)
Now is time to clone th repository</p>
`cd <your_ws>/src`<br>
`git clone https://github.com/nooshin-kohli/simulation.git`<br>
<p>There is no need to do all these steps all over again every time you want to use simulation. In the next part you will learn how to run this codes.</p>
### Run simulation
<p>Open new terminal</p>
`cd <your_ws>`<br>
`source devel/setup.bash`<br>
<p>To visualize robot in rviz:</p> 
`roslaunch first_leg urdf_rviz.launch`<br>
<p>To visualize robot in Gazebo:</p>
`roslaunch first_leg urdf_gazebo.launch`

