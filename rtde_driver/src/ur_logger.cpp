#include <ros/ros.h>
#include <rosbag/bag.h>
#include <iostream>
#include </home/antoon/catkin_ws/devel/include/rtde_driver/test_msg.h>
#include <ur_client_library/rtde/rtde_client.h>
#include <ur_client_library/ur/ur_driver.h>

const std::string OUTPUT_RECIPE = "/home/antoon/catkin_ws/src/rtde_driver/resources/test.txt";
const std::string INPUT_RECIPE = "/home/antoon/catkin_ws/src/rtde_driver/resources/test_inp.txt";

void get_vector6d(std::string name,std::unique_ptr<urcl::rtde_interface::DataPackage> &data_pkg,boost::array<double, 6UL> fdp_element) 
{
    urcl::vector6d_t temp;
    for (int i=0;i<6;i++)
    {
        fdp_element[i]=temp[i];
    };
}

void msg_write(std::unique_ptr<urcl::rtde_interface::DataPackage> &data_pkg,rtde_driver::test_msg &full_data_pkg)
{
    if(data_pkg->getData<double>("timestamp",full_data_pkg.timestamp)){};
    get_vector6d("actual_q",data_pkg,full_data_pkg.actual_q);
    /*if(data_pkg->getData<int>("",full_data_pkg.)){};
    if(data_pkg->getData<int>("",full_data_pkg.)){};
    if(data_pkg->getData<int>("",full_data_pkg.)){};
    if(data_pkg->getData<int>("",full_data_pkg.)){};
    if(data_pkg->getData<int>("",full_data_pkg.)){};
    if(data_pkg->getData<int>("",full_data_pkg.)){};
    if(data_pkg->getData<int>("",full_data_pkg.)){};
    if(data_pkg->getData<int>("",full_data_pkg.)){};
    if(data_pkg->getData<int>("",full_data_pkg.)){};
    if(data_pkg->getData<int>("",full_data_pkg.)){};
    if(data_pkg->getData<int>("",full_data_pkg.)){};
    if(data_pkg->getData<int>("",full_data_pkg.)){};
    if(data_pkg->getData<int>("",full_data_pkg.)){};
    if(data_pkg->getData<int>("",full_data_pkg.)){};
    if(data_pkg->getData<int>("",full_data_pkg.)){};*/
}

int main(int argc, char** argv) 
{
    ros::init(argc,argv,"ur_logger");
    ros::NodeHandle node;
    ros::Publisher pub;

    urcl::comm::INotifier notifier;
    const std::chrono::milliseconds READ_TIMEOUT{ 100 };

    urcl::rtde_interface::RTDEClient logger_client("192.168.56.102", notifier, OUTPUT_RECIPE, INPUT_RECIPE);
    logger_client.init();
    logger_client.start();
    pub = node.advertise<rtde_driver::test_msg>("logger_topic",1);
    while(ros::ok())
    {
        std::unique_ptr<urcl::rtde_interface::DataPackage> data_pkg = logger_client.getDataPackage(READ_TIMEOUT);
        if (data_pkg)
        {
            rtde_driver::test_msg full_data_pkg;
            msg_write(data_pkg,full_data_pkg);
            pub.publish(full_data_pkg);
        }
        ros::spinOnce();
    }
}