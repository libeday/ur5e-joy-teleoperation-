#pragma once
#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <cartesian_control_msgs/FollowCartesianTrajectoryActionGoal.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/types.h>
#include <ur_client_library/rtde/rtde_client.h>
#include <memory>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>

bool ur_script_start;

const std::string OUTPUT_RECIPE = "/home/antoon/catkin_ws/src/rtde_driver/resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "/home/antoon/catkin_ws/src/rtde_driver/resources/rtde_input_recipe.txt";

double delta_tcp_pose[6] = {0}; //[x, y, z, rx, ry, rz]

class TeleopRobot
{
public:
  TeleopRobot();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  ros::NodeHandle nh_;

  int x, y, z_up, z_dwn, rx, rz, ry;
  double x_scale_, y_scale_, z_scale_, rot_scale_;

  ros::Subscriber joy_sub_;
};

TeleopRobot::TeleopRobot()
{
  nh_.param("axis_x", x, 1);
  nh_.param("axis_y", y, 0);
  nh_.param("axis_z_up", z_up, 4);
  nh_.param("axis_z_dwn", z_dwn, 5);
  nh_.param("axis_rx", rx, 2);
  nh_.param("axis_ry", ry, 3);
  nh_.param("axis_rz", rz, 4);
  nh_.param("scale_y", y_scale_, y_scale_);
  nh_.param("scale_x", x_scale_, x_scale_);
  nh_.param("scale_z", z_scale_, z_scale_);
  nh_.param("scale_rot", rot_scale_, rot_scale_);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopRobot::joyCallback, this);
}

void TeleopRobot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  delta_tcp_pose[0] = x_scale_/100*joy->axes[1];
  delta_tcp_pose[1] = y_scale_/100*joy->axes[0];
  delta_tcp_pose[2] = (z_scale_/100*joy->buttons[4]) - (z_scale_/100*joy->buttons[5]);
  delta_tcp_pose[3] = rot_scale_/100*joy->axes[2];
  delta_tcp_pose[4] = rot_scale_/100*(joy->axes[3]);
  delta_tcp_pose[5] = rot_scale_/100*joy->axes[4];
  
  ROS_INFO("callback!!!! delta = [%f, %f, %f, %f, %f, %f]",delta_tcp_pose[0], delta_tcp_pose[1], delta_tcp_pose[2], delta_tcp_pose[3], delta_tcp_pose[4], delta_tcp_pose[5]);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "telejoy");

  std::string ROBOT_IP;
  ros::param::set("/ROBOT_IP",ROBOT_IP);

  urcl::comm::INotifier notifier;
  const std::chrono::milliseconds READ_TIMEOUT{ 100 };
  urcl::rtde_interface::RTDEClient my_client(ROBOT_IP, notifier, OUTPUT_RECIPE, INPUT_RECIPE);
  
  TeleopRobot telejoy;

  using namespace urcl;
  my_client.init();
  my_client.start();
  
  while (ros::ok())
  {
    
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = my_client.getDataPackage(READ_TIMEOUT);
    if (data_pkg) 
    {
      
      vector6d_t cur_tcp;
      
      if(data_pkg->getData<bool>("output_bit_register_64",ur_script_start))
      {
        ROS_INFO("reading bool_reg: %d", ur_script_start);
      }

      if(data_pkg->getData<urcl::vector6d_t>("actual_TCP_pose",cur_tcp)) 
      {
        if (ur_script_start)
        {
          for (int i=0; i<6; i++) 
          {
            my_client.getWriter().sendInputDoubleRegister(24+i,cur_tcp[i]);
          }
          my_client.getWriter().sendInputBitRegister(64,true);
        }

        for (int i=0; i<6; i++) 
        {
          if (delta_tcp_pose[i]!=0)
          {
            cur_tcp[i] += delta_tcp_pose[i];
            my_client.getWriter().sendInputDoubleRegister(24+i,cur_tcp[i]);
            delta_tcp_pose[i] = 0;
          }
        }
        ros::spinOnce();
      };
    } 
    else 
    {
      std::cout << "Could not get fresh data package from robot" << std::endl;
    }
    
  }
ros::spin();
  
}

