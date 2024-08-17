# Bot-with-Robotic-Arm

To view the bot in rviz and gazebo use
```sh
ros2 launch arm_bot_description display.launch.xml 
```

It should appear like this
![image](https://github.com/user-attachments/assets/962268ef-70f6-4bb1-bac1-003b7481a897)

## Controlling the bot using keyboard

First launch the bot in gazebo environment using

```sh
ros2 launch arm_bot_description bot.launch.py 
```

* Then Use the below command in terminal to control the bot from keyboard <br/>
```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/diffbot_base_controller/cmd_vel_unstamped
```
