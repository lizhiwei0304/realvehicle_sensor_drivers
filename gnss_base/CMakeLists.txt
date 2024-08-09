cmake_minimum_required(VERSION 3.5)
project(gnss_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(nav_msgs REQUIRED) # 添加组合导航标准消息包
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(geometry_msgs REQUIRED)

# 添加头文件搜索路径
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  /home/gnss_driver/install/gnss_sdk/include
)

# 添加静态库
add_library(gnss_messenger STATIC src/gnss_messenger.cpp)
target_link_libraries(gnss_messenger
  ${rclcpp_LIBRARIES} /home/gnss_driver/install/gnss_sdk/lib/libgnss_sdk.a
)

# 设置静态库的属性
set_property(TARGET gnss_messenger PROPERTY POSITION_INDEPENDENT_CODE ON)

# 添加可执行文件
add_executable(gnss_base_node src/gnss_base_node.cpp)
target_link_libraries(gnss_base_node
  gnss_messenger
  ${rclcpp_LIBRARIES}
  /home/gnss_driver/install/gnss_sdk/lib/libgnss_sdk.a
)

# 设置依赖项
ament_target_dependencies(gnss_messenger
  rclcpp nmea_msgs std_msgs sensor_msgs geometry_msgs controller_manager nav_msgs
)
ament_target_dependencies(gnss_base_node
  rclcpp nmea_msgs std_msgs sensor_msgs geometry_msgs controller_manager nav_msgs
)

# 安装可执行文件
install(TARGETS
  gnss_base_node
  DESTINATION lib/${PROJECT_NAME}
)

# 生成ament软件包
ament_package()