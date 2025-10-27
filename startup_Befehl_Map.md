
#Startup befehl python skript mit Kontext (um Roboter mit Map zu starten)
user@user-Alienware-Aurora-R9:~/ROS2$ ros2 launch go1_navigation navi_map_MPPI.launch.py


#startup tele operation
user@user-Alienware-Aurora-R9:~/ROS2$ ros2 run teleop_twist_keyboard teleop_twist_keyboard

# build des neuen Python Skript
user@user-Alienware-Aurora-R9:~/ROS2$ colcon build

#setup skript
source install/setup.bash

# run des polygon ablaufen Skript
user@user-Alienware-Aurora-R9:~/ROS2$ ros2 run polygon_nav polygon_explorer


# start des yolo detection Modells
user@user-Alienware-Aurora-R9:~/ROS2$ ros2 run polygon_nav yolo_processor

#start des person tracker
user@user-Alienware-Aurora-R9:~/ROS2$ ros2 run polygon_nav person_tracker
