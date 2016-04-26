1. Put mars.jpg into /usr/share/gazebo-2.2/media/materials/textures
2. Put mars folder into ~/.gazebo/models
3. Put gimbal_gazebo, gimbal_tracking into ~/(your catkin workspace)/src
   and catkin_make in (your catkin workspace)
4. 'roslaunch gimbal_gazebo gimbal.launch' for joystick control
5. 'roslaunch gimbal_tracking gimbal.launch' for gimbal tracking control