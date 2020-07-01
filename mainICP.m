% This script is to test that the ICP algorithm is working as expected.
% icp function is used in UKF Localization ICP.

clear all;
clc;
close all;

addpath('init_functions');
addpath('lidar_functions');
addpath('mapping_functions');
addpath('localization_functions');
addpath('generatedData')

load('lidarOutput.mat');

laser = getLidarParam();
             
pose.north = 1.5;
pose.east  = 0.5;
pose.psi   = pi/2;

% Show forward:
t1 = 1;
t2 = 20;

% Time Step 1:
[xe,ye]  = inverseLidarModel(pose,laser,lidarRanges(:,t1));
range1 = [xe';ye'];

% Time Step 2:
[xe2,ye2]  = inverseLidarModel(pose,laser,lidarRanges(:,t2));
range2 = [xe2';ye2'];

% The data points do not fit so well to the model points using LS-criterion
% Running the ICP-algorithm. Welsch criterion ( option 4)
% minimum increase to 20, maximum increase to 150 (iterations)

% How does Range time step 2 moves to range time step 1
[RotMat,TransVec,dataOut] = icp(range1,range2,10,100,4); % moves range 2 to range 1

figure(1)
scatter(range2(1,:),range2(2,:));
xlim([-2 10]);
ylim([-2 10]);
hold on;
scatter(range1(1,:),range1(2,:));
legend('Time step 2', 'Time step 1');
title('Before ICP');



%% Testing translation and rotation values %%%%%%%%%%%%%%%%

% Proving that it can work to propose better states:

% Time Step 1:
[xe,ye]  = inverseLidarModel(pose,laser,lidarRanges(:,t1));
range1 = [xe';ye'];

% Time Step 2 (transform):

% new = R*old + T
% Use translation and rotation information to move pose 

new_poses  = RotMat*([pose.east;pose.north]) +  TransVec;

pose.north   = new_poses(2);
pose.east    = new_poses(1);

rotated_degree = asin(RotMat(1,2));
pose.psi       = pose.psi + rotated_degree;

[xe2,ye2]  = inverseLidarModel(pose,laser,lidarRanges(:,t2));
range2 = [xe2';ye2'];


figure(2)
scatter(range2(1,:),range2(2,:));
xlim([-2 10]);
ylim([-2 10]);
hold on;
scatter(range1(1,:),range1(2,:));
legend('Time step 2', 'Time step 1');
title('Scan matching to propose states');


% If everything works as expected, the predict forward pose for step2
% should cause the inverseLidarModel to give the same readings
% Walls are at the same place. Assumption: Static surrounding.

% Perfect! It works great.
