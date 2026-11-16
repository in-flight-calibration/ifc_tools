cmake_minimum_required(VERSION 3.20)
project(ifc_tools)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ifc_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(px4_msgs REQUIRED)
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
    Eigen3::Eigen
    ifc_msgs::ifc_msgs__rosidl_typesupport_cpp
    ${OpenCV_LIBS}
    px4_msgs::px4_msgs__rosidl_typesupport_cpp
    rclcpp::rclcpp
    image_transport::image_transport
    cv_bridge::cv_bridge
    sensor_msgs::sensor_msgs
)

add_executable(mosaic examples/mosaic.cpp)
target_link_libraries(mosaic PRIVATE ifc_tools)

add_executable(terrain examples/terrain.cpp)
target_link_libraries(terrain PRIVATE ifc_tools)

install(
    TARGETS mosaic terrain
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    enable_testing()
    add_executable(ifc_tools_ring_buffer test/processing/test_ring_buffer.cpp)
    target_include_directories(ifc_tools_ring_buffer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    add_test(NAME ifc_tools_ring_buffer COMMAND ifc_tools_ring_buffer)
    add_executable(ifc_tools_interpolator test/processing/test_interpolator.cpp)
    target_include_directories(ifc_tools_interpolator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    add_test(NAME ifc_tools_interpolator COMMAND ifc_tools_interpolator)
    add_executable(ifc_tools_kalman_filter_1d test/filter/test_kalman_filter_1d.cpp)
    target_include_directories(ifc_tools_kalman_filter_1d PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    add_test(NAME ifc_tools_kalman_filter_1d COMMAND ifc_tools_kalman_filter_1d)
    add_executable(ifc_tools_terrain_grid test/processing/test_terrain_grid.cpp)
    target_include_directories(ifc_tools_terrain_grid PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(ifc_tools_terrain_grid PRIVATE Eigen3::Eigen)
    add_test(NAME ifc_tools_terrain_grid COMMAND ifc_tools_terrain_grid)
endif()

install(
    TARGETS ifc_tools
    EXPORT export_ifc_tools
)
install(
    DIRECTORY include/
    DESTINATION include
)

ament_export_targets(export_ifc_tools)
ament_export_dependencies(Eigen3 ifc_msgs OpenCV px4_msgs rclcpp image_transport cv_bridge sensor_msgs)

ament_package()
