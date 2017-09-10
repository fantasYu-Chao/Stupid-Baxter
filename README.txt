# Stupid-Baxter
#
# This project is designed cooperatively by Eugenio, Enrique, Chao, Penelope and Teng 
# to fulfill the pick and place task on a Baxter robot. The main contribution on code are in
# path: catkin_ws/src/vicon_bridge/src/
#
# This software can only be run in a computer runing Linux and with the pakages installed.
# To run the pick and place program using Baxter and the Vicon system, flowing steps have to be followed. 
#
# First ensure that the connection between the Vicon Host computer and Baxter Host computer is established.
#
# To do this, go to the connection panel on the upper right corner.
#
#
# If the vicon connection does not appears, we have to set up the network.
# 
# In Ubuntu, go to settings then Network to open the Network preference window.
# 
# Now click on options and select Ethernet to create the network.
# 
# Give the name of 'vicon' to the network and using the 'Manual' Method insert the addresses.
# 
# Then, click on 'Routes...' and check the box “Use this connection only for resources on its network”. 
# 
# Select 'ok' and save the network. 
# 
# The network between Vicon Host computer and Baxter Host computer is know established.
#
# With the Vicon connection established, open a new terminal window.
#
# Then source the catkin workspace and ros packages.
# $ source /home/picknplace/catkin_ws/devel/setup.bash
#
# Move to the catkin workspace, connect to Baxter and launch the Vicon Bridge.
# $ cd catkin_ws 
# $ . baxter.sh
# $roslaunch vicon_bridge vicon.launch
#
# Now the Vicon Bridge should be publishing the data from Vicon's models.
#
# Open a new terminal, move to the catkin workspace and connect to Baxter. 
# $ cd catkin_ws
# $ . baxter.sh
#
# If Baxter has not been enabled, enable Baxter.
# $ rosrun baxter_tools enable_robot.py -e
#
# Run the main program in the Vicon bridge.
# $ rosrun  vicon_bridge codebaxterZ.py
#
# From this point forward follow the instructions on the terminal.

