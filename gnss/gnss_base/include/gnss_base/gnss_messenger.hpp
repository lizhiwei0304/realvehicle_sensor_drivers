#ifndef GNSS_MESSENGER_HPP
#define GNSS_MESSENGER_HPP

#include <string>

#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>
#include <nav_msgs/Odometry.h>  //添加标准组合导航数据头文件
#include <sensor_msgs/Imu.h>

#include "gps_common/GPSFix.h" //添加gps头文件
#include "gnss_sdk/gnss_protocol/gnss_base.hpp"

#include <gps_common/conversions.h> //添加gps转换头文件 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h> //添加四元数转换头文件



namespace wescore
{
class GnssROSMessenger
{
public:
    explicit GnssROSMessenger(ros::NodeHandle *nh);
    GnssROSMessenger(GnssBase *gnss, ros::NodeHandle *nh);
    void SetupSubscription();
    void PublishStateToROS();

private:
    GnssBase *gnss_;
    ros::NodeHandle *nh_;

    ros::Publisher gps_publisher_;
    ros::Publisher imu_publisher_;
    ros::Publisher odom_publisher_; //添加组合导航话题 odom_publisher_

    ros::Time current_time_;
    int gpgga_send_ = 0, gprmc_send_ = 0, gpfpd_send_ = 0, gtimu_send_ = 0;
};
} // namespace wescore

#endif /* GNSS_MESSENGER_HPP */
