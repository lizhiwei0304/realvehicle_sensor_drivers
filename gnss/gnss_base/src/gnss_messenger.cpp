#include "gnss_base/gnss_messenger.hpp"


namespace wescore
{
GnssROSMessenger::GnssROSMessenger(ros::NodeHandle *nh) : gnss_(nullptr), nh_(nh)
{
}

GnssROSMessenger::GnssROSMessenger(GnssBase *gnss, ros::NodeHandle *nh) : gnss_(gnss), nh_(nh)
{
}

void GnssROSMessenger::SetupSubscription()
{
    // gps publisher
    gps_publisher_ = nh_->advertise<nmea_msgs::Sentence>("/hunter_1/nmea_sentence", 10);

    // imu publisher
    imu_publisher_ = nh_->advertise<sensor_msgs::Imu>("/imu/data",10);

    // 添加 odom publisher
    odom_publisher_ = nh_->advertise<nav_msgs::Odometry>("/xingwangyuda/odom", 10);
}

  /**
   * Get Geometry (UTM) point from GNSS position, assuming zero origin.########添加经纬度向UTM转换函数
   */

void GnssROSMessenger::PublishStateToROS()
{
    current_time_ = ros::Time::now();

    auto state = gnss_->GetGnssState();

    if (state.gpfpd_data[0] == '\0' || state.gtimu_data[0] == '\0' || state.gpgga_data[0] == '\0' || state.gprmc_data[0] == '\0')
    {
      if (state.gpfpd_data[0] == '\0'){std::cout << "gpfpd empty" << std::endl;}
      if (state.gtimu_data[0] == '\0'){std::cout << "gtimu empty" << std::endl;}
      if (state.gpgga_data[0] == '\0'){std::cout << "gpgga empty" << std::endl;}
      if (state.gprmc_data[0] == '\0'){std::cout << "gprmc empty" << std::endl;}
    }

    if ((gpfpd_send_+1)%state.loop_len <= state.gpfpd_get)
    {
        // $GPFPD
        nmea_msgs::Sentence gps_msg;
//        std::cout << "publish fpd" << std::endl;
        gps_msg.header.stamp = current_time_;
        gps_msg.sentence = state.gpfpd_data;
        std::cout << gps_msg.sentence << std::endl;

        // gps_publisher_.publish(gps_msg);
        //输出nav_msgs::msg::Odometry格式的odom_publisher_话题
        // "$GPFPD,0,3062.580,0.000,0.580,-0.202,0.00000000,0.00000000,0.00,0.000,0.000,0.000,0.000,0,0,00*5D";
        std::string odom_str;
        std::vector<std::string> odom_data;
        std::stringstream odom_ss(state.gpfpd_data);

        while (getline(odom_ss, odom_str, ','))
            odom_data.push_back(odom_str);
            
        if (odom_data.size() >= 16){ //判断gnss信息完整性
            nav_msgs::Odometry odom_msg;
            odom_msg.header.stamp = current_time_;
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";

            /**
            * Get Geometry (UTM) point from GNSS position, assuming zero origin.########添加经纬度向UTM转换函数
            */
            std::string zone; //unused
            double lat = std::atof(odom_data[6].c_str());
            double lon = std::atof(odom_data[7].c_str());
            double x,y;
            gps_common::LLtoUTM(lat, lon, y, x, zone); //经纬度转UTM

            odom_msg.pose.pose.position.x = x;
            odom_msg.pose.pose.position.y = y;
            odom_msg.pose.pose.position.z = std::atof(odom_data[8].c_str());

            /**
            * 进行四元数转换，同时对yaw角进行变换，使其正东向为0，正北向为90，正南向为-90度
            */
            double roll  = std::atof(odom_data[5].c_str());
            double pitch = std::atof(odom_data[4].c_str());
            double yaw   = std::atof(odom_data[3].c_str());

            if (yaw > 270 && yaw <360){
                yaw = -yaw + 450;
            } else{
                yaw = -yaw + 90;
            }
            std::cout << "yaw angle is : " << yaw << " degree." << std::endl;
            tf2::Quaternion quaternion;
            quaternion.setRPY(roll*M_PI / 180.0 , pitch*M_PI / 180.0 , yaw*M_PI / 180.0 );
            geometry_msgs::Quaternion quat_msg = tf2::toMsg(quaternion);

            odom_msg.pose.pose.orientation = quat_msg;

            odom_publisher_.publish(odom_msg);

        }else{std::cout << "\n\nskip once because of no full gpsdata\n\n" << std::endl;}
    }

    gpgga_send_ = state.gpgga_get;
    gprmc_send_ = state.gprmc_get;
    gtimu_send_ = state.gtimu_get;
    gpfpd_send_ = state.gpfpd_get;
}
} // namespace wescore
