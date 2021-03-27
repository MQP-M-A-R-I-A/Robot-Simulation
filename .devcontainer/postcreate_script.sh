# Make User using local computer user/group IDs
sudo usermod -u $USER_ID user
sudo groupmod -g $GROUP_ID user

# Source ROS stuff
echo "source /opt/ros/melodic/setup.bash" >> /home/user/.bashrc 
echo "source /home/user/catkin_ws/devel/setup.bash" >> /home/user/.bashrc

# Run ROSDep
rosdep update