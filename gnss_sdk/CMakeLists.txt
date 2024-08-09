cmake_minimum_required(VERSION 3.5)
project(gnss_sdk)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)

## Find packages in ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

## Specify additional locations of header files
include_directories(
  include
)

## Declare a C++ library
set(gnss_BASE_SRC
    src/gnss_protocol/gnss_base.cpp
    src/gnss_protocol/gnss_uart_parser.c
    src/async_io/async_serial.cpp
    src/async_io/async_can.cpp
    src/async_io/asyncio_utils.cpp
)
add_library(gnss_sdk STATIC ${gnss_BASE_SRC})
set_target_properties(gnss_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(gnss_sdk PUBLIC "-DASIO_ENABLE_OLD_SERVICES -DASIO_HAS_POSIX_STREAM_DESCRIPTOR")
target_link_libraries(gnss_sdk pthread ${rclcpp_LIBRARIES})
target_include_directories(gnss_sdk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/asio>
    $<INSTALL_INTERFACE:include>    
    PRIVATE src)

## Install targets and files
install(TARGETS gnss_sdk
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY asio/asio
  DESTINATION include
)

install(FILES asio/asio.hpp
  DESTINATION include
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
)

## Export information to other packages
ament_export_dependencies(rclcpp std_msgs)
ament_export_include_directories(include)
ament_export_libraries(gnss_sdk)
ament_package()