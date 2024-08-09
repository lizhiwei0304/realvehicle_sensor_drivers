#ifndef GNSS_MESSENGER_HPP
#define GNSS_MESSENGER_HPP

#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include <nmea_msgs/msg/sentence.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>  //添加标准组合导航数据头文件
#include "gnss_sdk/gnss_protocol/gnss_base.hpp"

using namespace std::chrono_literals;

namespace wescore
{
  class GnssROSMessenger : public rclcpp::Node
  {
  public:
    explicit GnssROSMessenger(GnssBase *gnss);
    void SetupSubscription();
    void PublishStateToROS();

    std::string GetPortTime() { return port_name; }
    int GetBaudRate() { return baud_rate; }

  private:
    GnssBase *gnss_;

    rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr gps_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_; //添加组合导航话题 odom_publisher_

    rclcpp::TimerBase::SharedPtr timer_;
    int gpgga_send_, gprmc_send_, gpfpd_send_, gtimu_send_;

    std::string port_name;
    int baud_rate;
  };
} // namespace wescore

#endif /* GNSS_MESSENGER_HPP */
