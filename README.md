# Wall-Follower

In order to run this program, you need to have a robot running ROS. Start ROSCORE, type the following command in the terminal of your ROS workspace:

rosrun <NAME_OF_PACKAGE> wall_follower.py

Once you execute this command, your robot should rotate and move towards the nearest wall and start moving along it. The robot will always move with its left side adjacent to the
wall that it is following.
