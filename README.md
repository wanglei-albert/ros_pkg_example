# ros_pkg_example
show how-to code to use some ros packages  

## depends package
ros-noetic-joint-trajectory-controller:  
	sudo apt-get install ros-noetic-joint-trajectory-controller  
ros-noetic-rqt-joint-trajectory-controller:  
	sudo apt-get install ros-noetic-rqt-joint-trajectory-controller  
dynamixel_msgs:  
	git clone https://github.com/arebgun/dynamixel_motor  

# RUN
	source devel/setup.bash
## ros_controller
	roslaunch dhrobot_controller arm_ros_control.launch  
	rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller --force-discover  
## 

	
