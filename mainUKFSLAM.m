% This script runs UKF SLAM using lidar measurements feedback.
% We are using lidar measurements as "states" in measurement update.
% Map gets constructed over time while simultaneously the robot is
% localising.

clc;clear;close all;

%  UKF SLAM
addpath('generatedData');
addpath('init_functions');
addpath('lidar_functions');
addpath('mapplot_functions');
addpath('localization_functions');
addpath('mapping_functions');
addpath('generatedData')

% Data inputs:
load('lidarOutput.mat'); % Lidar range measurements
load('vel_inputs.mat');  % Velocity inputs

% Get robot parameters
param = getRobotParameters();

% Get lidar parameters
lidar = getLidarParam();


n  = 3;              % number of states
nm = lidar.numScans; % number of measurements
N = length(v);       % time length

% R and Q is tune to make it go the expected trajectory.
R = 70*eye(nm);
Q = 1e-4*diag([0.1,0.1,0.001]);

%Make some room for the filter
Pp  = zeros(n,n,N+1);
Pf  = zeros(n,n,N);
mup = zeros(n,N+1);  
muf = zeros(n,N);

%setup initial state PDF
mup(:,1)    = [1.5; 2; pi/2];
Pp(:,:,1)   = 0.01*eye(n);

rh = [];

% Set up mapping variables:

% Initialise empty map for mapping:
grid            = initOccupancyGrid(); % Grid used to fill probabilities (for log-odds)
[Fnc,z]         = plotInit(grid);
mappad          = z; % for initial log-odds
new_map         = initOccupancyGrid(); % Extra grid used as new map every iteration

% intiliase log-odds:
logP = 0;
ProbMap = z; % Temporary map (Updates every iteration for localisation)
% 
% 
% writerObj           = VideoWriter('UKFSLAMtuned.mp4','MPEG-4');
% writerObj.FrameRate = 10;

% Open the video writer
% open(writerObj);

for t=1:N-1
    
    %delete(rh);
    %rh = drawRobot(mup(:,1:t));
    

  
    % Varying covariance involve with lidar measurements: 
    % Finding the index of max ranges from lidar measurement:
    logic_maxRange =  lidarRanges(:,t) > lidar.maxRange-1;
    id_maxRange = find(logic_maxRange == 1) ;
    
    % Changing covariances on max ranges:
    for index = 1:length(id_maxRange)
        % When max range is detected, increase R as it the measurement is
        % less "trustable".
        R(id_maxRange(index),id_maxRange(index)) = 80; % More Noisy (high R)
        
    end
    
    % Form new constructed map for localisation:
    
    % If probability of map is above 0.5, take it as occupied (1) 
    % If probability of map is below or equal 0.5, take it as unoccupied(0)
    new_map.Z = double(ProbMap(1:end-1,1:end-1) > 0.5);
    
    
    % Run the Unscented Kalman Filter with new map every iteration:
    [mu,P]    = UnscentedTransformUpdate(new_map,lidar,n,[mup(:,t);zeros(nm,1)],blkdiag(Pp(:,:,t),R));
    
    error = lidarRanges(:,t) - mu(n+1:end);
    muf(:,t)  = mu(1:n) + P(1:n,n+1:end)*(P(n+1:end,n+1:end)\(error));
    Pf(:,:,t) = P(1:n,1:n) - P(1:n,n+1:end)*(P(n+1:end,n+1:end)\P(n+1:end,1:n));
    
    pose.north  = muf(1,t);
    pose.east   = muf(2,t);
    pose.psi    = muf(3,t); 
    
    % Use filtered states to do mapping:
    
    % Inverse lidar model gets end points of lidar hits
    [xe,ye]  = inverseLidarModel(pose,lidar,lidarRanges(:,t));
    
    % Take in current grid, draw line, fill in information to form new grid.
    new_grid = bresenhamLineAlgo(grid,pose,xe,ye);
    
    % New cells with probability information
    z(1:end-1,1:end-1) = new_grid;
    
    % Calculate log odds:
    logP = log(z./(1-z)) + logP - log(mappad./(1-mappad));
    
    % Calculate probability of occupancy:
    ProbMap = 1./(1+exp(-logP));
   
    % Update occupancy grid map as map grows
    %plotUpdate(Fnc,ProbMap);
%     F = getframe(gcf) ;
%     writeVideo(writerObj, F);
    
    %Predict forward
    pred = @(x) [x(1:n); robotDiscKinematics(x(1:n),v(:,t),param) + x(n+1:end)];

    [mu,P]      = UnscentedTransform(pred,[muf(:,t);zeros(n,1)],blkdiag(Pf(:,:,t),Q));
    mup(:,t+1)  = mu(n+1:end);
    Pp(:,:,t+1) = P(n+1:end,n+1:end);


    
end  


% close(writerObj);
% fprintf('Video generated\n');  



