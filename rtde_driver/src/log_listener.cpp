#include <ros/ros.h>
#include </home/antoon/catkin_ws/devel/include/rtde_driver/test_msg.h>

void chatterCallback(const rtde_driver::test_msg::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->actual_q[2]);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "log_listener");
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("logger_topic", 1000, chatterCallback);
    ros::spin();

}