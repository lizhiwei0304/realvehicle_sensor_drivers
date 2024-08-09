#include "gnss_base/gnss_messenger.hpp"

namespace wescore
{
  GnssROSMessenger::GnssROSMessenger(GnssBase *gnss) : Node("gnss_odom"),
                                         gnss_(gnss),
                                         gpgga_send_(0),
                                         gprmc_send_(0),
                                         gpfpd_send_(0),
                                         gtimu_send_(0)
  {
  }

  void GnssROSMessenger::SetupSubscription()
  {
    this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 115200);

    this->get_parameter("port_name", port_name);
    this->get_parameter("baud_rate", baud_rate);

    // gps publisher
    gps_publisher_ = this->create_publisher<nmea_msgs::msg::Sentence>("/scout/nmea_sentence", 10);

    // imu publisher
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

    // 添加 odom publisher
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/xingwangyuda/odom", 10);

    timer_ = this->create_wall_timer(10ms, std::bind(&GnssROSMessenger::PublishStateToROS, this));
  }

  void GnssROSMessenger::PublishStateToROS()
  {
    auto current_time_ = rclcpp::Clock().now();

    auto state = gnss_->GetGnssState();

    if (state.gpfpd_data[0] == '\0' || state.gtimu_data[0] == '\0' || state.gpgga_data[0] == '\0' || state.gprmc_data[0] == '\0')
    {
      if (state.gpfpd_data[0] == '\0'){std::cout << "gpfpd empty" << std::endl;}
      if (state.gtimu_data[0] == '\0'){std::cout << "gtimu empty" << std::endl;}
      if (state.gpgga_data[0] == '\0'){std::cout << "gpgga empty" << std::endl;}
      if (state.gprmc_data[0] == '\0'){std::cout << "gprmc empty" << std::endl;}
    }

    if ((gpfpd_send_ + 1) % state.loop_len <= state.gpfpd_get)
    {
      // $GPFPD
      nmea_msgs::msg::Sentence gps_msg;
      std::cout << "publish fpd" << std::endl;
      gps_msg.header.stamp = current_time_;
      gps_msg.sentence = state.gpfpd_data;
      std::cout << gps_msg.sentence << std::endl;

      // gps_publisher_->publish(gps_msg);

      //输出nav_msgs::msg::Odometry格式的odom_publisher_话题
      // "$GPFPD,0,3062.580,0.000,0.580,-0.202,0.00000000,0.00000000,0.00,0.000,0.000,0.000,0.000,0,0,00*5D";
      std::string odom_str;
      std::vector<std::string> odom_data;
      std::stringstream odom_ss(state.gpfpd_data);

      while (getline(odom_ss, odom_str, ','))
        odom_data.push_back(odom_str);

      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = current_time_;
      odom_msg.header.frame_id = "odom";
      odom_msg.child_frame_id = "base_link";

      odom_msg.pose.pose.position.x = std::atof(odom_data[6].c_str());
      odom_msg.pose.pose.position.y = std::atof(odom_data[7].c_str());
      odom_msg.pose.pose.position.z = std::atof(odom_data[8].c_str());

      odom_msg.pose.pose.orientation.x = std::atof(odom_data[3].c_str());
      odom_msg.pose.pose.orientation.y = std::atof(odom_data[4].c_str());
      odom_msg.pose.pose.orientation.z = std::atof(odom_data[5].c_str());
      odom_msg.pose.pose.orientation.w = 1;

      odom_publisher_->publish(odom_msg);
    }
    // if ((gtimu_send_ + 1) % state.loop_len <= state.gtimu_get)
    // {
    //   // $GTIMU
    //   std::string str;
    //   std::vector<std::string> data;
    //   std::stringstream ss(state.gtimu_data);

    //   while (getline(ss, str, ','))
    //     data.push_back(str);

    //   // "$GTIMU,0,879.720,-0.0133,-0.0599,-0.0623,0.0185,0.0041,0.9958,33.0*47";
    //   sensor_msgs::msg::Imu imu_msg;
    //   imu_msg.header.stamp = current_time_;
    //   imu_msg.orientation_covariance = {-1, -1, -1, -1, -1, -1, -1, -1, -1};

    //   imu_msg.angular_velocity.x = std::atof(data[3].c_str());
    //   imu_msg.angular_velocity.y = std::atof(data[4].c_str());
    //   imu_msg.angular_velocity.z = std::atof(data[5].c_str());

    //   imu_msg.linear_acceleration.x = std::atof(data[6].c_str()) * 9.8;
    //   imu_msg.linear_acceleration.y = std::atof(data[7].c_str()) * 9.8;
    //   imu_msg.linear_acceleration.z = std::atof(data[8].c_str()) * 9.8;

    //   imu_publisher_->publish(imu_msg);
    // }
    gpgga_send_ = state.gpgga_get;
    gprmc_send_ = state.gprmc_get;
    gtimu_send_ = state.gtimu_get;
    gpfpd_send_ = state.gpfpd_get;
  }
} // namespace wescore
