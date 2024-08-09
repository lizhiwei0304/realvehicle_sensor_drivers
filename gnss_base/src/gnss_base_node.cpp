#include "rclcpp/rclcpp.hpp"
#include <string>
#include "gnss_base/gnss_messenger.hpp"

using namespace wescore;

void DetachRobot(int signal) {
  std::terminate();
}

int main(int argc, char **argv)
{
    // setup ROS node
    rclcpp::init(argc, argv);

    std::signal(SIGINT, DetachRobot);

    // instantiate a robot object
    GnssBase robot;

    std::shared_ptr<GnssROSMessenger> node_ptr = std::make_shared<GnssROSMessenger>(&robot);

    std::cout <<"Using UART to talk with the robot" <<std::endl;
    node_ptr->SetupSubscription();

    // connect to port
    robot.Connect(node_ptr->GetPortTime(), node_ptr->GetBaudRate());

    rclcpp::spin(node_ptr);

    return 0;
}
