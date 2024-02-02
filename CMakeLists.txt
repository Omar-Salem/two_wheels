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
find_package(behaviortree_cpp_v3 REQUIRED)

set(dependencies
        rclcpp
        sensor_msgs
        geometry_msgs
        nav_msgs
        angles
        tf2_geometry_msgs
        behaviortree_cpp_v3
)

add_library(go_straight_bt_node SHARED src/go_straight.cpp)
add_library(is_clear_bt_node SHARED src/is_clear.cpp)
add_library(turn_bt_node SHARED src/turn.cpp)

list(APPEND plugin_libs
        go_straight_bt_node
        is_clear_bt_node
        turn_bt_node
)

foreach (bt_plugin ${plugin_libs})
    ament_target_dependencies(${bt_plugin} ${dependencies})
    target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach ()

add_executable(${PROJECT_NAME} src/main.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(DIRECTORY include/
        DESTINATION include/
)

install(TARGETS
        ${plugin_libs}
        ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})


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
        DIRECTORY behavior_tree_xml config launch rviz urdf worlds
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
