% This script runs mapping with known poses.
clear;
close all;
clc;

addpath('mapplot_functions');
addpath('mapping_functions'); % all of the necessary functions do mapping with known poses
addpath('generatedData');
addpath('init_functions');

% Data generated given by mainDataGeneration.m
load('lidarOutput.mat'); %  Lidar ranges (simulated with noise)
load('statesOutput.mat'); % States (Known poses) 

%Data expected is states variable (x) and lidar measurements (lidarRanges)
N   = x(1,:); %North
E   = x(2,:); %East
psi = x(3,:); %Psi

% Ground truth map for comparison:
map = getMapParam();

% Initialise empty map for mapping:
grid            = initOccupancyGrid();
[Fnc,z]         = plotInit(grid);
mappad          = z; % for initial log-odds

% Get lidar parameters
lidar = getLidarParam();

% intiliase log-odds:
logP = 0;

% create the video writer with 10 fps
% writerObj = VideoWriter('mapping.mp4','MPEG-4');
% writerObj.FrameRate = 10;
% 
% % open the video writer
% open(writerObj);

 for t = 1:length(N)-1
     
             
    pose.north =  N(t);
    pose.east  =  E(t) ;
    pose.psi   =  psi(t);
    
    % Inverse lidar model gets end points of lidar hits
    [xe,ye]  = inverseLidarModel(pose,lidar,lidarRanges(:,t));
    
    % Take in current grid, draw line, fill in information to form new grid.
    new_grid = bresenhamLineAlgo(grid,pose,xe,ye);
    
    % New cells with probability information
    z(1:end-1,1:end-1) = new_grid;
    
    % Calculate log odds:
    logP = log(z./(1-z)) + logP - log(mappad./(1-mappad));
    
    % Calculate probability of occupancy:
    P = 1./(1+exp(-logP));
   
    % Update occupancy grid map as map grows
    plotUpdate(Fnc,P);
    rh = drawRobot(x(:,1:t));
    pause(0.05);
    % write the frames to the video
    % F = getframe(gcf) ;
    % writeVideo(writerObj, F);
    
    delete(rh);
    
    
 end
 
% close(writerObj);
% fprintf('Video generated\n');   