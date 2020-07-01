% This script runs UKF Localisation using ICP as measurements feedback.

clc;clear;close all;

%  UKF LOCALIZATION
%  Assuming known map.
addpath('generatedData');
addpath('init_functions');
addpath('lidar_functions');
addpath('mapplot_functions');
addpath('simulateTruth_functions');
addpath('localization_functions');
addpath('mapping_functions');

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

ns = 3; % number of states
nm = ns; % number of measurements


load('lidarOutput.mat'); % Lidar range measurements
load('vel_inputs.mat');  % Velocity inputs

N = length(v);

R = diag([0.1,0.1,1.5]); %0.1*eye(nm);
Q = 0.005*eye(ns);

%Make some room for the filter
Pp  = zeros(ns,ns,N+1);
Pf  = zeros(ns,ns,N);
mup = zeros(ns,N+1);  
muf = zeros(ns,N);

%setup initial state PDF
mup(:,1)    = zeros(ns,1);
mup(:,1)    = [1.5; 2; pi/2];
mup(:,2)    = [1.5; 2; pi/2];
Pp(:,:,1)   = 0.01*eye(ns);


meas = @(x) [x(1:ns); x(1:ns) + x(ns+(1:nm))];
pred = @(x) [x(1:ns); robotDiscKinematics(x(1:ns),v(:,t),param) + x(ns+1:end)];

ICPmeas = [1.5;2;pi/2];% + 0.001*randn(ns,1);

for t=1:N-1
    
    delete(rh);
    rh = drawRobot(mup(:,1:t));
    
    pose.north  = mup(1,t);
    pose.east   = mup(2,t);
    pose.psi    = mup(3,t); % radian

    %Run the Unscented Kalman Filter
    meas = @(x) [x(1:ns); x(1:ns) + x(ns+(1:nm))];
    pred = @(x) [x(1:ns); robotDiscKinematics(x(1:ns),v(:,t),param) + x(ns+1:end)];

    %Update with new measurement
    [mu,P]      = UnscentedTransform(meas,[mup(:,t);zeros(nm,1)],blkdiag(Pp(:,:,t),R));
    muf(:,t)    = mu(1:ns) + P(1:ns,ns+1:end)*(P(ns+1:end,ns+1:end)\(ICPmeas - mu(ns+1:end)));
    Pf(:,:,t)   = P(1:ns,1:ns) - P(1:ns,ns+1:end)*(P(ns+1:end,ns+1:end)\P(ns+1:end,1:ns));

    %Predict forward
    [mu,P]      = UnscentedTransform(pred,[muf(:,t);zeros(ns,1)],blkdiag(Pf(:,:,t),Q));
    mup(:,t+1)  = mu(ns+1:end);
    Pp(:,:,t+1) = P(ns+1:end,ns+1:end);

    
    % Time Step 1:
    [xe,ye]  = inverseLidarModel(pose,lidar,lidarRanges(:,t));
    range1 = [xe';ye'];

    % Time Step 2:
    [xe2,ye2]  = inverseLidarModel(pose,lidar,lidarRanges(:,t+1));
    range2 = [xe2';ye2'];

    % Running the ICP-algorithm. Welsch criterion ( option 4)
    % minimum increase to 20, maximum increase to 150 (iterations)

    % How does Range time step 2 moves to range time step 1
    [RotMat,TransVec,dataOut] = icp(range1,range2,20,150,4); % moves range 2 to range 1


    % new = R*old + T
    % Use translation and rotation information to move pose 

    new_poses  = RotMat*([pose.east;pose.north]) +  TransVec;

    pose.north   = new_poses(2);
    pose.east    = new_poses(1);

    rotated_degree =  asin(RotMat(1,2));
    pose.psi       =  pose.psi + rotated_degree;
    
    ICPmeas = [pose.north; pose.east; pose.psi];
    
    
end  





