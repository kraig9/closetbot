# Closetbot
Requires packages to run, I typically install them with Debian. I am using ROS Humble, so typically

sudo apt-get ros-humble-{package} (ROS Humble requires Ubuntu Jellyfish, 22.04)


To run the main scripts, run something like
ros2 launch /home/{your home}/ws_dexarm/src/closetbot/launch/gazebolaunch.launch.py
