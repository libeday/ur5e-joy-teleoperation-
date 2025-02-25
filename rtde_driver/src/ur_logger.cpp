#include <ros/ros.h>
#include <rosbag/bag.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include </home/antoon/catkin_ws/devel/include/rtde_driver/test.h>
#include </home/antoon/catkin_ws/devel/include/rtde_driver/test_msg.h>
#include <ur_client_library/rtde/rtde_client.h>
#include <ur_client_library/ur/ur_driver.h>

const std::string OUTPUT_RECIPE = "/home/antoon/catkin_ws/src/rtde_driver/resources/log_output_no_reg.txt";
const std::string INPUT_RECIPE = "/home/antoon/catkin_ws/src/rtde_driver/resources/rtde_input_recipe.txt";

void get_vector6d(std::string name,std::unique_ptr<urcl::rtde_interface::DataPackage> &data_pkg,boost::array<double, 6UL> &fdp_element) 
{
    urcl::vector6d_t temp;
    if(data_pkg->getData<urcl::vector6d_t>(name,temp)){
        //ROS_INFO_STREAM(name);
    };
    for (int i=0;i<6;i++)
    {
        fdp_element[i]=temp[i];
    };
}

void get_vector3d(std::string name,std::unique_ptr<urcl::rtde_interface::DataPackage> &data_pkg,boost::array<double, 3UL> &fdp_element) 
{
    urcl::vector3d_t temp;
    if(data_pkg->getData<urcl::vector3d_t>(name,temp)){
        //ROS_INFO_STREAM(name);
    };
    for (int i=0;i<3;i++)
    {
        fdp_element[i]=temp[i];
    };
}

void get_vector6i(std::string name,std::unique_ptr<urcl::rtde_interface::DataPackage> &data_pkg,boost::array<int32_t, 6UL> &fdp_element) 
{
    urcl::vector6int32_t temp;
    if(data_pkg->getData<urcl::vector6int32_t>(name,temp)){
        //ROS_INFO_STREAM(name);
    };
    for (int i=0;i<6;i++)
    {
        fdp_element[i]=temp[i];
    };
}

std::string write_index(std::string name, int index) // ("name_",64)=>"name_64"
{
    std::string num = std::to_string(index);
    return name + num;
}

int main(int argc, char** argv) 
{
    ros::init(argc,argv,"ur_logger");
    ros::NodeHandle node;
    ros::Publisher pub;
    ros::Publisher tpub;

    urcl::comm::INotifier notifier;
    const std::chrono::milliseconds READ_TIMEOUT{ 100 };

    urcl::rtde_interface::RTDEClient logger_client("192.168.56.102", notifier, OUTPUT_RECIPE, INPUT_RECIPE);
    logger_client.init();
    logger_client.start();
    tpub = node.advertise<rtde_driver::test>("logger_topic",10);
    ros::Duration d(1./100.);
    
    while(ros::ok())
    {
        std::unique_ptr<urcl::rtde_interface::DataPackage> data_pkg = logger_client.getDataPackage(READ_TIMEOUT);
        if (data_pkg)
        {
            rtde_driver::test test_pkg;

            if(data_pkg->getData<double>("timestamp",test_pkg.timestamp)){};
            get_vector6d("actual_q",data_pkg,test_pkg.actual_q);
            get_vector6d("target_q",data_pkg,test_pkg.target_q);
            get_vector6d("target_qd",data_pkg,test_pkg.target_qd);
            get_vector6d("target_qdd",data_pkg,test_pkg.target_qdd);
            get_vector6d("target_current",data_pkg,test_pkg.target_current);
            get_vector6d("target_moment",data_pkg,test_pkg.target_moment);
            get_vector6d("actual_qd",data_pkg,test_pkg.actual_qd);
            get_vector6d("actual_current",data_pkg,test_pkg.actual_current);
            get_vector6d("joint_control_output",data_pkg,test_pkg.joint_control_output);
            get_vector6d("actual_TCP_pose",data_pkg,test_pkg.actual_TCP_pose);
            get_vector6d("actual_TCP_speed",data_pkg,test_pkg.actual_TCP_speed);
            get_vector6d("actual_TCP_force",data_pkg,test_pkg.actual_TCP_force);
            get_vector6d("target_TCP_pose",data_pkg,test_pkg.target_TCP_pose);
            get_vector6d("target_TCP_speed",data_pkg,test_pkg.target_TCP_speed);
            if(data_pkg->getData<uint64_t>("actual_digital_input_bits",test_pkg.actual_digital_input_bits)){};
            get_vector6d("joint_temperatures",data_pkg,test_pkg.joint_temperatures);
            if(data_pkg->getData<double>("actual_execution_time",test_pkg.actual_execution_time)){};
            if(data_pkg->getData<int32_t>("robot_mode",test_pkg.robot_mode)){};
            get_vector6i("joint_mode",data_pkg,test_pkg.joint_mode);
            if(data_pkg->getData<int32_t>("safety_mode",test_pkg.safety_mode)){};
            get_vector3d("actual_tool_accelerometer",data_pkg,test_pkg.actual_tool_accelerometer);
            if(data_pkg->getData<double>("speed_scaling",test_pkg.speed_scaling)){};
            if(data_pkg->getData<double>("target_speed_fraction",test_pkg.target_speed_fraction)){};
            if(data_pkg->getData<double>("actual_momentum",test_pkg.actual_momentum)){};
            if(data_pkg->getData<double>("actual_main_voltage",test_pkg.actual_main_voltage)){};
            if(data_pkg->getData<double>("actual_robot_voltage",test_pkg.actual_robot_voltage)){};
            if(data_pkg->getData<double>("actual_robot_current",test_pkg.actual_robot_current)){};
            get_vector6d("actual_joint_voltage",data_pkg,test_pkg.actual_joint_voltage);
            if(data_pkg->getData<uint64_t>("actual_digital_output_bits",test_pkg.actual_digital_output_bits)){};
            if(data_pkg->getData<uint32_t>("runtime_state",test_pkg.runtime_state)){};
            get_vector3d("elbow_position",data_pkg,test_pkg.elbow_position);
            get_vector3d("elbow_velocity",data_pkg,test_pkg.elbow_velocity);
            if(data_pkg->getData<uint32_t>("robot_status_bits",test_pkg.robot_status_bits)){};
            if(data_pkg->getData<uint32_t>("safety_status_bits",test_pkg.safety_status_bits)){};
            if(data_pkg->getData<uint32_t>("analog_io_types",test_pkg.analog_io_types)){};
            if(data_pkg->getData<double>("standard_analog_input0",test_pkg.standard_analog_input0)){};
            if(data_pkg->getData<double>("standard_analog_input1",test_pkg.standard_analog_input1)){};
            if(data_pkg->getData<double>("standard_analog_output0",test_pkg.standard_analog_output0)){};
            if(data_pkg->getData<double>("standard_analog_output1",test_pkg.standard_analog_output1)){};
            if(data_pkg->getData<double>("io_current",test_pkg.io_current)){};
            if(data_pkg->getData<uint32_t>("euromap67_input_bits",test_pkg.euromap67_input_bits)){};
            if(data_pkg->getData<uint32_t>("euromap67_output_bits",test_pkg.euromap67_output_bits)){};
            if(data_pkg->getData<double>("euromap67_24V_voltage",test_pkg.euromap67_24V_voltage)){};
            if(data_pkg->getData<double>("euromap67_24V_current",test_pkg.euromap67_24V_current)){};
            if(data_pkg->getData<uint32_t>("tool_mode",test_pkg.tool_mode)){};
            if(data_pkg->getData<uint32_t>("tool_analog_input_types",test_pkg.tool_analog_input_types)){};
            if(data_pkg->getData<double>("tool_analog_input0",test_pkg.tool_analog_input0)){};
            if(data_pkg->getData<double>("tool_analog_input1",test_pkg.tool_analog_input1)){};
            if(data_pkg->getData<int>("tool_output_voltage",test_pkg.tool_output_voltage)){};
            if(data_pkg->getData<double>("tool_output_current",test_pkg.tool_output_current)){};
            if(data_pkg->getData<double>("tool_temperature",test_pkg.tool_temperature)){};
            if(data_pkg->getData<uint32_t>("output_bit_registers0_to_31",test_pkg.output_bit_registers0_to_31)){};
            if(data_pkg->getData<uint32_t>("output_bit_registers32_to_63",test_pkg.output_bit_registers32_to_63)){};
            if(data_pkg->getData<uint32_t>("input_bit_registers0_to_31",test_pkg.input_bit_registers0_to_31)){};
            if(data_pkg->getData<uint32_t>("input_bit_registers32_to_63",test_pkg.input_bit_registers32_to_63)){/*ROS_INFO("ok");*/};
            tpub.publish(test_pkg);
        } 
        d.sleep();
        ros::spinOnce();
    }
}