Unzip Hexapod.zip
run "blender -b -P script.py" (make sure you have blender installed using snap)
create two terminal windows with ros2 sourced using "source /opt/ros/jazzy/setup.bash"
run "ros2 launch hexapod_display_launch.py"
in another window set up python virtual environment that has pybullet installed
run "python3 pybullet_to_rviz.py"
