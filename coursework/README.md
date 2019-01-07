# Group 11 Robot Cluedo Instructions
## Authors:
Jacob Bickerstaff <br>
Jacob Holland <br>
Mantas Kilopkinas <br>
Aimilia Ntokou <br>

### Setup and run

##### file name:
robotStatus
###### configurations required:
1. Go to the \__init__ definition and alter the coordinates for the centre of the map and the narrow entrance.
2. Run the set up the program by running the following:
   roslaunch coursework coursework_setup.launch
3. (if required) estimation position in Rviz, if initial pose in rviz and gazebo do not match.
4. Then run the program by running the following in the terminal:
   roslaunch coursework coursework_run.launch


### Changing world and map files

There are two ways to change the world and map files for the robot simulation to use:

1. Modify the coursework_setup.launch file so the map_file and world_file arguments contain full path for the world and map files.
2. specify world and map files as arguments before runing the coursework_setup.launch file.
e.g. `roslaunch coursework coursework_setup.launch map_file:=$HOME/catkin_ws/src/coursework/maps/demo1.yaml world_file:=$HOME/catkin_ws/src/coursework/src/worlds/demo1.world`