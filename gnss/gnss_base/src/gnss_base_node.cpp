#include <ros/ros.h>
#include <string>
#include "gnss_base/gnss_messenger.hpp"

using namespace wescore;

void DetachRobot(int signal) {
  std::terminate();
}

int main(int argc, char **argv)
{
    // setup ROS node
    ros::init(argc, argv, "gnss_odom");
    ros::NodeHandle node(""), private_node("~");

    std::signal(SIGINT, DetachRobot);

    // instantiate a robot object
    GnssBase robot;
    GnssROSMessenger messenger(&robot, &node);

    // fetch parameters before connecting to robot
    std::string port_name;
    int baud_rate;
    private_node.param<std::string>("port_name", port_name, std::string("/dev/gps"));
    private_node.param<int>("baud_rate", baud_rate, 115200);

    // connect to port
    robot.Connect(port_name, baud_rate);
    ROS_INFO("Using UART to talk with the robot");
    messenger.SetupSubscription();

    // publish gnss state at max 100Hz
    ros::Rate rate_100hz(100); // 100Hz
    while (ros::ok())
    {
        ros::spinOnce();
        messenger.PublishStateToROS();
        rate_100hz.sleep();
    }

    return 0;
}
