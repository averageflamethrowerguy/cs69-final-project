# How to run this project
Note: These steps assume that you are currently running a docker container with ROS and this package already installed. 

## Running the program (Gazebo)
1. With the Docker Daemon running, open a new terminal and open a BASH terminal in the ROS container `docker-compose exec ros bash`.
2. Run `source /opt/ros/melodic/setup.bash`
3. Start the simulator by running `roslaunch simple_flocking simple_world.launch` and view the simulator openning your browser to `localhost:8080/vnc.html` and clicking `Connect`.
4. Open a new terminal and, once again, open a bash terminal in the ROS container by running `docker-compose exec ros bash`.
5. Finally, to run the program and see your polygon recreated by the robot in the simulator, run `roslaunch simple_flocking simple_flocking`.

## Change Parameters (Gazebo)
To change the parameters for this simulation, edit `~/catkin_ws/src/simple_flocking/launch/simple_flocking.launch` and modify the XML under the heading **Flocking Motion Node Parameters**.

## Running the program (StageRos)
1. With the Docker Daemon running, open a new terminal and open a BASH terminal in the ROS container `docker-compose exec ros bash`.
2. Run `source /opt/ros/melodic/setup.bash`
3. See the virtual linux GUI by openning your browser to `localhost:8080/vnc.html` and clicking `Connect`.
4. In the same terminal window from earlier, run `roslaunch simple_flocking simple_flocking_2d`.

## Change Parameters (StageRos)
To change the parameters for this simulation, edit `~/catkin_ws/src/simple_flocking/launch/simple_flocking_2d.launch` and modify the XML under the heading **Flocking Motion Node Parameters**.

To change the layout of the world in the StageRos simulation, modify the `~/catkin_ws/src/simple_flocking/worlds/empty_world.world` file.