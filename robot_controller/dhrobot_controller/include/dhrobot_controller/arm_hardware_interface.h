#ifndef ARM_HW_INTERFACE_H
#define ARM_HW_INTERFACE_H

#include <ros/ros.h>
#include <urdf/model.h>


#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>


#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_msgs/JointState.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/ListControllers.h>

#include <pthread.h>
#include <time.h>

class ArmHWInterface :public hardware_interface::RobotHW
{
public:
    ArmHWInterface();
    void read(const dynamixel_msgs::JointStateConstPtr& msg);
    void publishCmd();
    ros::NodeHandle getNode();
private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_cmd_interface;

    double cmd[5];
    double pos[5];
    double vel[5];
    double eff[5];


    controller_manager_msgs::ListControllersRequest list_req;
    controller_manager_msgs::ListControllersResponse list_resp;

    bool loaded_flag;
    
    ros::NodeHandle _n;
    ros::Publisher pub_cmd[5];
    std_msgs::Float64 cmd_msg[5];
    ros::Time start_time_;
    ros::Duration start_dur_;

    ros::Subscriber sub_js[5];
    ros::ServiceClient client_controller_list;
};

#endif