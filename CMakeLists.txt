cmake_minimum_required(VERSION 3.8)
project(two_wheels)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()


set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_executable(bump_go src/bump_go.cpp)
ament_target_dependencies(bump_go rclcpp sensor_msgs geometry_msgs nav_msgs angles tf2_geometry_msgs)

install(TARGETS
        bump_go
        DESTINATION lib/${PROJECT_NAME})


set(HW_INTERFACE_LIB "diff_drive_hw_interface")
add_library(
        ${HW_INTERFACE_LIB}
        SHARED
        firmware/arduino_tb6612fng.cpp
)
ament_target_dependencies(
        ${HW_INTERFACE_LIB}
        hardware_interface
        rclcpp
        rclcpp_lifecycle
)
# prevent pluginlib from using boost
target_compile_definitions(${HW_INTERFACE_LIB} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(hardware_interface ${HW_INTERFACE_LIB}.xml)

# install(
#         TARGETS
#         ${HW_INTERFACE_LIB}
#         RUNTIME DESTINATION bin
#         ARCHIVE DESTINATION lib
#         LIBRARY DESTINATION lib
# )


install(
        DIRECTORY config launch rviz urdf worlds
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
