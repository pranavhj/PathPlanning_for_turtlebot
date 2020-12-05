This gives the instructions to run the ros program for navigation of the turtlebot using Astar with holonomic constraints

For the video provided the inputs were initialized but will have to be provided when you run the code


The initialized inputs for video 1 are
Enter x of start co-ordinates:4.5
Enter y of start co-ordinates:4.5
Enter theta of start co-ordinates:0
Enter x of Goal co-ordinates:-4.5
Enter y of Goal co-ordinates:-2.5
Enter left wheel speed:36
Enter right wheel speed:-27
Enter clearance value: 45

The initialized inputs for video 2 are
Enter x of start co-ordinates:4.5
Enter y of start co-ordinates:3
Enter theta of start co-ordinates:0
Enter x of Goal co-ordinates:0
Enter y of Goal co-ordinates:3
Enter left wheel speed:36
Enter right wheel speed:-27
Enter clearance value:45

VERY IMP.............
System Requirements- Ubuntu 16.04 or higher, gazebo 7 or higher installed, check if gazebo_ros package is present by using roscd gazebo_ros. turtlebot3, turtlebot3_msgs, turtlebot3_simulations packages should be installed and the workspace they are present in should be sourced
You can download above packages from link
https://github.com/pranavhj/Turtlebot_repos/upload/master

Instructions to run the code
1. Extract the files into a catkin_ws
2. source ~/catkin_ws/devel/setup.bash
3. Run in a teminal
	catkin_make
If all goes well you can proceed
4. In a teminal run
export TURTLEBOT3_MODEL=waffle
5. try 
roslaunch p3p4_group35_ros_python turtlebot3_p3p4.launch
this will open turtlebot in the location specified in the above launch file. It can be changed by going into the file (Make sure the spawning position is close to the position that is given as a start to Astar. There should not be any obstacle between spawning position and the start position given to Astar. The code will automatically make the turtlebot go to the start position of the Astar if above condition is true)

5.2. If 5. does not work, try
export WORKON_HOME=/home/pranav/.virtualenvs
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
source /usr/local/bin/virtualenvwrapper.sh
 5. should definately work after this. if it does not, it means there is a problem with installation

6. Then run in another terminal
rosrun p3p4_group35_ros_python moveto.py 

Provide the parameters you want or the ones given above to simulate the above given cases
Also after a red line appears on the window where exploration is going on, means that the path planning has completed. You have to press any key on the keyboard to proceed. After this watch the turtlebot track the path found.

 


