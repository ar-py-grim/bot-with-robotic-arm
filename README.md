# Bot-with-Robotic-Arm

To view the bot in rviz and gazebo use
```sh
ros2 launch
```

## Controlling the bot using keyboard

* To spawn the bot in gazebo and open rviz2 use <br/>
```sh
ros2 launch arm
```

* Then Use the below command in terminal to control the bot from keyboard <br/>
```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped
```
