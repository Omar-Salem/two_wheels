cmake_minimum_required(VERSION 3.8)
project(two_wheels)

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
        angles
        tf2_geometry_msgs)
set(targets
        bump_go)

foreach (d ${deps})
    find_package(${d} REQUIRED)
endforeach ()

foreach (t ${targets})
    add_executable(${t} src/${t}.cpp)
    ament_target_dependencies(${t} ${deps})
endforeach ()

#add_executable(bump_go src/bump_go.cpp)
#ament_target_dependencies(bump_go ${deps})


install(TARGETS
        ${targets}
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

install(
        DIRECTORY config launch rviz urdf worlds
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
