% This script runs UKF Localisation using lidar measurements feedback.
% We are using lidar measurements as "states" in measurement update.

clc;clear;close all;

%  UKF LOCALIZATION
%  Assuming known map.
addpath('generatedData');
addpath('init_functions');
addpath('lidar_functions');
addpath('mapplot_functions');
addpath('localization_functions');
addpath('mapping_functions');
addpath('generatedData')

% Get robot parameters
param = getRobotParameters();

% Get lidar parameters
lidar = getLidarParam();

% Get known map parameters
map = getMapParam();

% Ready a figure
hh   	= readyFigure(map);
hmap    = hh{1};
hs      = hh{2};
ht      = hh{3};
rh      = hh{4};

n  = 3; % number of states
nm = lidar.numScans; % number of measurements


load('lidarOutput.mat'); % Lidar range measurements
load('vel_inputs.mat');  % Velocity inputs

N = length(v);

% R and Q is tune to make it go the expected trajectory.
R = 2*eye(nm);
Q = 1e-4*eye(n);

%Make some room for the filter
Pp  = zeros(n,n,N+1);
Pf  = zeros(n,n,N);
mup = zeros(n,N+1);  
muf = zeros(n,N);

%setup initial state PDF
mup(:,1)    = [1.5; 2; pi/2];
Pp(:,:,1)   = 0.01*eye(n);



for t=1:N-1
    
    delete(rh);
    rh = drawRobot(mup(:,1:t));
    
    pose.north  = mup(1,t);
    pose.east   = mup(2,t);
    pose.psi    = mup(3,t); % radian

    pred = @(x) [x(1:n); robotDiscKinematics(x(1:n),v(:,t),param) + x(n+1:end)];
  
    % Varying covariance involve with lidar measurements:
    
    % Finding the index of max ranges from lidar measurement:
    logic_maxRange =  lidarRanges(:,t) > lidar.maxRange-0.5;
    id_maxRange = find(logic_maxRange == 1) ;
    
    % Changing covariances on max ranges:
    for index = 1:length(id_maxRange)
        % When max range is detected, increase R as it the measurement is
        % less "trustable".
        R(id_maxRange(index),id_maxRange(index)) = 10; % More Noisy (high R)
        
    end
    
    
    % Run the Unscented Kalman Filter

    [mu,P]    = UnscentedTransformUpdate(map,lidar,n,[mup(:,t);zeros(nm,1)],blkdiag(Pp(:,:,t),R));
    
    error = lidarRanges(:,t) - mu(n+1:end);
    muf(:,t)  = mu(1:n) + P(1:n,n+1:end)*(P(n+1:end,n+1:end)\(error));
    Pf(:,:,t) = P(1:n,1:n) - P(1:n,n+1:end)*(P(n+1:end,n+1:end)\P(n+1:end,1:n));
    
    %Predict forward
    [mu,P]      = UnscentedTransform(pred,[muf(:,t);zeros(n,1)],blkdiag(Pf(:,:,t),Q));
    mup(:,t+1)  = mu(n+1:end);
    Pp(:,:,t+1) = P(n+1:end,n+1:end);


    
    
end  





