% Localisation using Particle Filters
% Global relocalisation is done at the start. 
% A lot of particles at unoccupied spaces.

clc;clear;close all;

addpath('init_functions');
addpath('lidar_functions');
addpath('mapplot_functions');
addpath('localization_functions');
addpath('generatedData')

% Load lidar ranges and velocity inputs from path trajectory:
load('vel_inputs.mat');
load('lidarOutput.mat');

% Get robot parameters
param = getRobotParameters();

% Get lidar parameters
lidar = getLidarParam();

% Get map parameters
map = getMapParam();

% Ready a figure
hh   	= readyFigure(map);
hmap    = hh{1};
hs      = hh{2};
ht      = hh{3};
rh      = hh{4};


% Generate robot trajectory with velocity inputs
N = length(v);  % time
M = 20000;      % Number of Particles
n = 3;          % Number of states

%Initialisation of particles for localisation:
x = zeros(n,M,N+1);
x = globalParticles(x,map,M); % this function set particles all over the map

%Initialisation needed for resampling:
idx         = 1:M;
M_heuristic = M/2;
lw_lidar    = zeros(M,1);
Q           = 0.05*eye(n);


h = [];
a = [];

% create the video writer with 1 fps
% writerObj = VideoWriter('PF_localisation.mp4','MPEG-4');
% writerObj.FrameRate = 10;
% 
% % open the video writer
% open(writerObj);

for t=1:N
    
   % NAIVE IMPLEMENTATION OF PARTICLES SAMPLE CHANGE. (KLD-SAMPLING NOT DONE):
   % Better way is to figure out some sort of heuristic to do this
   
    if t == 10
                
        M   = 1500; % new amount of particles
        idx = int32(linspace(1,length(lw_lidar),M));
        x   = x(:,idx,:);
        lw_lidar = zeros(length(M),1);
        
    end
    
    if t == 20
                
        M   = 500; % new amount of particles
        idx = int32(linspace(1,length(lw_lidar),M));
        x   = x(:,idx,:);
        lw_lidar = zeros(length(M),1);
        
    end

   for i  = 1:M % going through all particles:
       

        
        pose.north = x(1,i,t);
        pose.east  = x(2,i,t);
        pose.psi   = x(3,i,t);

        % Compute lidar likelihood for each particle:
        lw_lidar(i)  = lidarModel(lidarRanges(:,t),map,lidar,pose);
        

   end % finish going through all particles
    
   % Normalising log-weigths
   mlw = max(lw_lidar);
   lw_lidar  = lw_lidar - mlw - log(sum(exp(lw_lidar-mlw)));
   
   if getNeff(exp(lw_lidar)) < M_heuristic % Effective resampling
       
        idx = resampleSystematic(lw_lidar);
    
   end
   
   
   x(:,:,t+1)  = robotDiscKinematics(x(:,idx,t),v(:,t),param) + sqrtm(Q)*randn(n,M);
   


   a = scatter(x(2,:,t),x(1,:,t),10);
   pause(0.01);
  
%    % write the frames to the video
%    F = getframe(gcf) ;
%    writeVideo(writerObj, F);
   delete(a);
   
    
end

    
% close(writerObj);
% fprintf('Video generated\n');




