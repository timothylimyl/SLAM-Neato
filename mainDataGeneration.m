% This script generates simulated lidar data according to the velocity
% inputs given. Lidar data generated has added noise.

clc;clear;close all;

addpath('init_functions');
addpath('lidar_functions');
addpath('mapplot_functions');
addpath('localization_functions');

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


% Simulate a noisy lidar scan to act as 'truth'
% lidarNoisyScan0 = simulateNoisyLidarScan(map,lidar);

% Generate robot trajectory with velocity inputs
N = 1600; %time
x       = zeros(3,N+1);
x(:,1)  = [1.5; 2; pi/2]; %North,east.psi

v = zeros(2,N);

% straight right
v(:,1:450) = v(:,1:450)+[100;100];

% turn to go up
v(:,451:500) = v(:,451:500)+[97;100];

% straight up
v(:,501:750) = v(:,501:750)+[100;100];

% turn to go left
v(:,751:805) = v(:,751:805)+[97;100];

% straight left
v(:,806:1250) = v(:,806:1250)+[100;100];

% turn to go down
v(:,1251:1300) = v(:,1251:1300)+[97;100];

% straight down
v(:,1301:1600) = v(:,1301:1600)+[100;100];


v = v*1e-3;

lidarRanges = zeros(lidar.numScans,N); % Lidar data from robot scan with added noise
h = [];

for t=1:N
    
    rh = drawRobot(x(:,1:t));
    
%     vstr   	= input('\n next velocity in [mm/sec]\n>> ','s');
%     v(:,t)	= str2num(vstr)'*1e-3;
    pose.north = x(1,t);
    pose.east  = x(2,t);
    pose.psi   = x(3,t); %State is in radian
    
    lidarRanges(:,t) = simulateNoisyLidarScan(map,lidar,pose);

    x(:,t+1) = robotDiscKinematics(x(:,t),v(:,t),param);
    
    h = updateScanPlotNE(h,lidar,pose,lidarRanges(:,t));
    
    pause(0.01);
    delete(rh);
    
end

    


