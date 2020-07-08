# Autonomy Software

This repo contains the Autonomy software stack for the current iteration of the MRDT Rover designed to compete at the University Rover Challenge. The software is developed to run on a Jetson TX2 development board.


## Architecture
The architecture is broken up into three categories:
1. Drivers - These are objects who wrap functionality of various data points from the rover as well as other core interfacing components within the Autonomy system (such as logging and cameras).
2. Algorithms - These are the core pieces of logic that perform the various computational taks that Autonomy requires, such as Haversine math, PID controls and Search pattern.
3. Tests - These are unit tests designed to test the basic functionality of any algorithm developed within the Autonomy system. These should be designed to easily test any changes made to an algorithm and should serve as a way to verify if changes broke functionality.

![Architecture Diagram](architecture.png)


