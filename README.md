``check_urdf /PATH/TO/URDF``

`colcon build --packages-select two_wheels && source install/setup.bash && ros2 launch two_wheels sim_two_wheels.launch.py`

Gotchas
An <inertia> element within each <link> element must be properly specified and configured.