# Neato SLAM (University of Newcastle MCHA4000 Module, G6)

![intro](images/mapping.gif)

## Introduction
---

The goal of this project is to implement a Simultaneous Localisation and Mapping (SLAM) solution for a [NEATO Vacuum Cleaner Robot](https://www.neatorobotics.com/au/). The project goal was to SLAM algorithm can be implemented onto an on-board microcontroller
in the Neato Robot via an interface with MATLAB code. This project was changed to a full simulation based project as access to the Neato Robot was hindered due to unforeseen circumstances caused by the global pandemic.

What is SLAM? To understand SLAM, we have to understand localisation and mapping first. Localisation is figuring where in the world is the robot given a map while mapping is figuring out the map of the surrouding environment of the robot.
SLAM deals with the problem of constructing a map of an unknown environment while simultaneously figuring out the location of the robot. If we do not have a map, how would we figure out the location of the robot (Localisation)? On the other hand,
if we do not know where is the robot, how would we map the surrounding environment to begin with (Mapping)? This is why SLAM is commonly known as a chicken-and-egg problem. In order to solve the SLAM problem, we can alternate between localisation and mapping
in search for the best estimation of the pose of the robot and map as time progress. This is usually done with the help of using many probabilistic approaches such as bayes filters which is use to
update the probability of the location of the robot (states) as more information are made known. 

The sensors used on the Neato Robot are encoders and Light Detection and Ranging (LIDAR). Encoders are used to calculate the number of rotation of the wheels while LIDAR sends out laser light and measuring the reflection of the laser 
with a sensor to compute the distance ranges. It is worth noting that sensors are corrupted with noise due to internal hardware limitation and the changes in the surrounding environment such as a bump on the road or fiction changes.
Therefore, probabilistic approaches are vital to ensure that information is consistently updated through time as a feedback mechanism as we can never fully trust sensor readings.


## Code Guide and explanation
----

The code guide section is divided into 3 main sections namely Mapping, Localisation, SLAM. Each section will have thoroughly explanation of the code involved and its location. Technical details will be shared too.

### Mapping

### Localisation

### SLAM




## Dependencies

As of 1st of July 2020, code successfully run on MATLAB R2019B with in-buiilt standard libraries. Author is not aware of any extra library needed. The only program necessary to run the whole code base is MATLAB.


