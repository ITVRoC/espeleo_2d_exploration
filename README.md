# espeleo_2d_exploration
2D Autonomous exploration package for EspeleoRobo. This package uses a Laser Scanner and Gmappping SLAM. Navigation Stack Navfn global planner is used.
Navigation can be done either via Navigation Stack Local Planner or vector field control.


### Dependencies:
- move_base (https://github.com/ros-planning/navigation)
- gmapping (https://github.com/ros-perception/slam_gmapping)

To launch EspeleoRobo with the navigation stack configuration, run ```roscore``` and open a CoppeliaSim scene with a robot equipped with a 2D Laser Scanner publishing to the /scan topic.

In a new terminal, run:
```
roslaunch espeleo_vrep_simulation espeleo_sim.launch 
```
Run the following script to publish /odom topic and correctly setup the tf tree:
```
python <path_to_catkin_ws>/src/espeleo_locomotion/script/odom.py 
```
In a new terminal, run:
```
roslaunch espeleo_2d_exploration 2d_exploration.launch 
```

A new Rviz window will open, showing the map published to the ```/map``` topic. To send a goal position to the robot, click on the 2D Nav Goal and click on the map at the desired location and orientation.
