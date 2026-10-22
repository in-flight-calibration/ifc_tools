cmake_minimum_required(VERSION 3.20)
project(ifc_tools)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(ifc_tools INTERFACE)

target_include_directories(ifc_tools INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(ifc_tools INTERFACE
    ${OpenCV_LIBS}
    rclcpp::rclcpp
    image_transport::image_transport
    cv_bridge::cv_bridge
    sensor_msgs::sensor_msgs
)

add_executable(mosaic examples/mosaic.cpp)
target_link_libraries(mosaic PRIVATE ifc_tools)

install(
    TARGETS mosaic
    DESTINATION lib/${PROJECT_NAME}
)

install(
    TARGETS ifc_tools
    EXPORT export_ifc_tools
)
install(
    DIRECTORY include/
    DESTINATION include
)

ament_export_targets(export_ifc_tools)
ament_export_dependencies(OpenCV rclcpp image_transport cv_bridge sensor_msgs)

ament_package()
