# vectorfield_stack
Navigation control algorithms based on artificial vector fields



This ROS stack ...




## Available packages


### distancefield

This package contains a vector field based navigation algorithm. The vector field is designed to make a robot follow a given curve. The method is currently implemented in 3 dimensions. The strategy is based on the minimum distance between the robot and the curve. The main theory that supports this implementation is published in the following paper:

[1] Adriano M. C. Rezende, Vinicius M. Goncalves and Luciano C. A. Pimenta, **Constructive Time-Varying Vector Fields for Robot Navigation,** in IEEE Transactions on Robotics, doi: 10.1109/TRO.2021.3093674.

The implementation also incorporates the ability to deviate from detected obstacles.



### robotsim

This package provides simple implementations of several robots. The objective of this package is to enable simple examples of the use of the control packages of this stack.



### ground_robot



### quad_robot



### examples