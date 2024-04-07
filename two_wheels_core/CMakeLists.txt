cmake_minimum_required(VERSION 3.8)
project(two_wheels_core)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()


set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)

set(deps
        rclcpp
        sensor_msgs
        geometry_msgs
        nav_msgs
        nav2_msgs
        tf2
        tf2_ros
        two_wheels_interfaces
        rclcpp_action
        nav2_costmap_2d
        nav2_map_server
        slam_toolbox
        visualization_msgs
        nav2_util
        std_msgs
        angles
        tf2_geometry_msgs
        map_msgs)
set(targets
        bump_go)

foreach (d ${deps})
    find_package(${d} REQUIRED)
endforeach ()

foreach (t ${targets})
    add_executable(${t} src/${t}.cpp)
    ament_target_dependencies(${t} ${deps})
endforeach ()


install(TARGETS
        ${targets}
        DESTINATION lib/${PROJECT_NAME})


set(HW_INTERFACE_LIB "hw_interface")
add_library(
        ${HW_INTERFACE_LIB}
        SHARED
        firmware/${HW_INTERFACE_LIB}/DiffDrive.cpp
)
ament_target_dependencies(
        ${HW_INTERFACE_LIB}
        hardware_interface
        rclcpp
        rclcpp_lifecycle
        two_wheels_interfaces
)
install(
        TARGETS
        ${HW_INTERFACE_LIB}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

#The arguments to the pluginlib_export_plugin_description_file command are:
#The package with the base class, i.e. polygon_base.
#The relative path to the Plugin Declaration xml, i.e. plugins.xml.
pluginlib_export_plugin_description_file(hardware_interface ${HW_INTERFACE_LIB}.xml)

install(
        DIRECTORY config include launch maps rviz urdf worlds
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
