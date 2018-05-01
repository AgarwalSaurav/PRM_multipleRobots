# Kinematic simulations using Klampt

### Execution:
```
cd simTests
python kinematicSim.py simpleWorld.xml
```

### Robot files

  The folder `mobile_robots` contain the descriptions of the following robots:
1. kobuki: It has the base of TurtleBot. It is a differential drive system with
   two degrees of freedom. Can be forced as a holonomic robot with three degrees
   of freedom.
2. turtlebot: Differential drive robot with two degrees of freedom.
3. R2-D2: Can be used as a differential drive robot.
4. sphero: It is a three degrees of freedom robot. However, it can be forced as
   a six DoF system as done in the samples.

### Simulation files

   The folder `simTests` contains files to perform simple kinematic simulations.
   1. simpleWorld.xml: Contains information on which robot to use and the size
      of the terrain. One should modify this file (or make a copy) to change the type of robot.
   2. kinematicSim.py: Creates visualization for kinematic simulations. This is
      the main template for simulating trajectories and collision checking.
   3. mathUtils.py: Basic math utility functions.
   4. buildWorld.py: To be used for adding walls or rooms to the environment.
    
   The folder `simTests/kinematics` contains wrapper functions for setting up
   the configuration of robots.
  
