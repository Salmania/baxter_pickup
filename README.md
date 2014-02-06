To Setup:
1. Install Rethink Robotics Baxter Packages: https://github.com/RethinkRobotics
2. Clone the repo:
   $git clone https://github.com/PeterJeffris/baxter_pickup.git
3. Add the networking configureation to your bashrc:
   $echo "source ~/PATH_TO_CATKIN_WORKSPACE/src/baxter_pickup/baxter_env_setup.sh" >> ~/.bashrc
   And edit that baxter_env_setup script to have the correct path prefixes. 
4. Source your bashrc:
   $source ~/.bashrc
5. Go to the top of the catkin workspace:
   $roscd baxter_pickup/../..
6. Make the package:
   $catkin make
7. Launch the baxter and moveit servers:
   $roslaunch baxter_pickup baxter_moveit_setup.launch
8. Launch the rviz server:
   $roslaunch baxter_pickup baxter_rviz.launch
9. Run the servoing executable:
   $rosrun baxter_pickup viz_servo 