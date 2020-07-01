# Neato SLAM (University of Newcastle MCHA4000 Module, G6)

![intro](images/mapping.gif)

## Introduction
---

The goal of this project is to implement a Simultaneous Localisation and Mapping (SLAM) solution for a [NEATO Vacuum Cleaner Robot](https://www.neatorobotics.com/au/). The initial project goal was to implement the SLAM algorithm onto an on-board microcontroller
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

The code guide section is divided into 4 main sections namely Data generation, Mapping with known poses, Localisation, SLAM. Each section will have thoroughly explanation of the code involved and its location. Technical details will be shared too.

All the main scripts to run data generation, localisation, mapping and slam are all `main*.m `files while the folders of functions contains the necessary auxiliaries scripts which path are added automatically in the `main*.m` scripts.

### Data Generation

Running `mainDataGeneration.m` will simulate a robot moving around given map. The lidar is assume to be in the centre of rotation of the robot, raycasting allows us to compute lidar range measurements. It can be seen in the animation below that
there are added noise into the lidar rays which is very important to simulate real lidar sensors. Lidar measurement noise was added by using a gaussian distribution on rays that hits obstacles,  potential short readings and random ranges
was also added. Lidar simulation scripts can be found in folder `lidar_functions` which consist of `scanNE.c` for raycasting and `simulateNoisyLidarScan.m` for adding noise.


![data](images/data_generated.gif)

The robot goes around with manual velocity inputs being fed into the kinematic model which can be found in `init_functions/robotDiscKinematics.m`. The map was done using the application Paint (free in-built Windows application) and converting
into binary values by `mapplot_functions/loadMapFromImage.m`. It is important that the image is converted to a binary matrix representation then we can represent value 1 as occupied and 0 as unoccupied. The code is set up in a way where
we have the flexibility to easily add obstacles, change the structure of the map and everything else will still run the same.

The `init_functions` folder also consist of all of the necessary initialisation for lidar parameters (`getLidarParam.m`),map parameters (`getMapParam.m`), robot process model parameters (`getRobotParameters.m`) and occupancy grid (`initOccupancyGrid.m`).

The lidar ranges and states generated are stored as `.mat` files in the folder `data_generated`.

### Mapping with known poses

Mapping with known poses is a problem where the robot has to map its surrounding environment assuming that we know the precise location of the robot at every time step. This is usually impossible due to sensor noise as previously mentioned.
However, the mapping algorithm used are exactly the same. Running `mainMapping.m` with move the robot around the map while mapping it. The mapping is using log-odds (good [reference](https://www.cs.cmu.edu/~16831-f14/notes/F14/16831_lecture06_agiri_dmcconac_kumarsha_nbhakta.pdf),
implemented in `line 57 of mainMapping.m`. Auxiliary scripts for mapping can be found in the folder `mapping_functions`. 


Firstly, we need to compute the end points location of the lidar measurements which is done in `mapping_functions/inverseLidarModel.m`. Then, use a line algorithm used to trace the ray from the starting point (location of robot)
to the end points return by `inverseLidarModel.m`. Bresenham Line Algorithm `mapping_functions/bresenhamLineAlgo.m` was used.  `bresenhamLineAlgo.m` outputs a newly updated grid with probabilities of occupancy for individual cells that the
line passes through (unoccupied) and where it ends/hits (occupied). Note that max ranges of lidar are filtered out as it is unnecessary risk in the case of this application as the lidar may return max range even if there is an obstacle 
ahead of it due to sensor failure. `mapping_functions/plotInit.m` initialises the occupancy grid map while `mapping_functions/plotUpdate.m` updates the occupancy grid map in every iteration.

Result of mapping algorithm can be seen in the comparison below:


Actual Map                |  Map from algorithm   
:-------------------------:|:-------------------------:|
![map](images/map.jpg)   | ![mapalgo](images/mapalgo.jpg) 





### Localisation

#### Particle Filter

Given the map, we need to figure out how to localise the robot (figure out where is the robot). At the initial few seconds, we have no clue on the initial position of the robot in the map, it could be anywhere that is unoccupied with any
orientation. Therefore, to solve this problem commonly known as the "Kidnapped Vehicle Problem", we need an algorithm that does global localisation. This can be done using a Particle Filter. In the picture below, shows the random initialisation
of particles distributed throughout the whole map using the function `localization_functions/globalParticles.m`. `globalParticles.m` only put particles in unoccupied cells of any map given to it.

![pf](images/global_particles.jpg)

The lidar likelihood model is set up in `localization_functions/lidarModel.m`, compute the log weights of every particle proposed. The particles are then resampled using the weights computed using the function `localization_functions/resampleSystematic.m`
and applying effective resampling using the function `localization_functions/getNeff.m` which was recommended by a paper . 

Run `mainLocalization_PF.m` to observe particle filter algorithm being run, animation to be expected:

![ukf](images/PF_localization.gif)



#### Unscented Kalman Filter (UKF)

Run `mainUKFLocalization.m` to observe using UKF for localization. The main functions used to implement UKF is in the unscented transform for the measurement update and prediction step (`UnscentedTransform.m` and `UnscentedTransformUpdate.m`).
`UnscentedTransformUpdate.m` uses `update.m` to update the lidar measurements by using the predicted pose (North,East,Yaw) in raycasting, the raycast readings are compared to the actual lidar measurements. Most of the time was spent tuning
the measurement covariances (parameter R) and process noise (Q) to get the UKF to work. The value of Q and R was adjusted till the robot manages to go around the whole map as expected.

![ukf](images/UKF_localization.gif)





### SLAM




## Dependencies

As of 1st of July 2020, code successfully run on MATLAB R2019B with in-buiilt standard libraries. Author is not aware of any extra library needed. The only program necessary to run the whole code base is MATLAB.


