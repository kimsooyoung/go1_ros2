

```
sudo apt install ros-eloquent-joy -y
cbp ros2_unitree_legged_msgs && roseloq
cbp unitree_legged_real && roseloq
cbp unitree_joy_cmd && roseloq
```

```
ros2 run unitree_legged_real ros2_udp highlevel
ros2 run unitree_legged_real ros_state_helper
ros2 launch unitree_joy_cmd eloquent_joy_cmd.launch.py
ros2 run joy joy_node
```



# TODO
[] keyboard cmd
[] low level cmd
[] camera sub - composition
[] ROS 2 Sim - Ignition Gazebo
