% This script runs FastSLAM 1.0. It uses Rao-Blackwellized Particle Filter
% Each particle contains an individual map.

clc;clear;close all;

addpath('init_functions');
addpath('lidar_functions');
addpath('mapplot_functions');
addpath('localization_functions');
addpath('generatedData')

% Load lidar ranges from path trajectory:
load('lidarOutput');
% Load velocity inputs:
load('vel_inputs.mat');

% Get robot parameters
param = getRobotParameters();

% Get lidar parameters
lidar = getLidarParam();

% Get empty occupancy grid:

% FORM AN EMPTY MAP WITH CELLS:
grid = initOccupancyGrid();

% Initialise the map for plotting (function handle used by plotUpdate)
[Fnc,z]         = plotInit(grid);
mappad          = z; % for initial log-odds

N = length(lidarRanges(1,:)); % time
M = 100;  % Number of Particles
n = 3;    % Number of states

%Initialisation of particles for localisation:
x = zeros(n,M,N+1);

% Assumption: You know where is the first position cause u place it there.
x(1,:,1) = 1.5 + 0.001*randn(M,1);
x(2,:,1) = 2 + 0.001*randn(M,1);
x(3,:,1) = (pi/2) + 0.001*randn(M,1);

%Initialisation needed for resampling:
M_heuristic = M/2;
lw_lidar    = zeros(M,1);
Q           = eye(n)*0.00001;  % 0.001 = ~1000particles

% Initialisation needed for Grid FastSlam
logP = zeros(grid.Nnorth+1,grid.Neast+1,M);
P    = zeros(grid.Nnorth+1,grid.Neast+1,M);
idx  = 1:M; % initialising resampling index (needed for effective resampling)
model_grid = initOccupancyGrid(); % Extra grid used for updating lidar and optic model.


h = [];
a = [];

% create the video writer with 1 fps
% writerObj = VideoWriter('solutionFastSLAM.mp4','MPEG-4');
% writerObj.FrameRate = 10;
% 
% %open the video writer
% open(writerObj);

for t=1:N
    

   for i  = 1:M % going through all particles:
        
        pose.north = x(1,i,t);
        pose.east  = x(2,i,t);
        pose.psi   = x(3,i,t);
        
        [xe,ye]  = inverseLidarModel(pose,lidar,lidarRanges(:,t));

        % Take in current grid, draw line, fill in information to form new grid.
        new_grid = bresenhamLineAlgo(grid,pose,xe,ye);

        z(1:end-1,1:end-1) = new_grid;

        % Each particle has its own map.
        % logP(:,:,idx(i)) ensures that map follows along with the
        % particle that is resampled.
        
        % Calculate log odds:
        logP(:,:,i) = log(z./(1-z)) + logP(:,:,idx(i)) - log(mappad./(1-mappad));

        % Calculate probability of occupancy:
        P(:,:,i) = 1./(1+exp(-logP(:,:,i)));
        
        % newly constructed map to be used for localising:
        model_grid.Z   =  double(P(1:end-1,1:end-1,i) > 0.6);
        
        % Compute lidar likelihood for each particle:
        lw_lidar(i)  = lidarModel(lidarRanges(:,t),model_grid,lidar,pose);
        

   end % finish going through all particles
    
   % Normalising log-weigths
   mlw = max(lw_lidar);
   lw_lidar  = lw_lidar - mlw - log(sum(exp(lw_lidar-mlw)));
   
   if getNeff(exp(lw_lidar)) < M_heuristic % Effective resampling
       
        idx = resampleSystematic(lw_lidar);
    
   end

    x(:,:,t+1)  = robotDiscKinematics(x(:,idx,t),v(:,t),param) + sqrtm(Q)*randn(n,M);


    % Update occupancy grid map as map grows
    % Picking the highest weight particle for visualisation
    [~,id] = max(exp(lw_lidar));
    plotUpdate(Fnc,P(:,:,id));
    rh = drawRobot(x(:,id,1:t));
    pause(0.01);
    % write the frames to the video
%     F = getframe(gcf) ;
%     writeVideo(writerObj, F);
    
    delete(rh);
 
  
end


% close(writerObj);
% fprintf('Video generated\n');
% 

  
