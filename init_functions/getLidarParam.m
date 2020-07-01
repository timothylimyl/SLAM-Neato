function lidar = getLidarParam()

% Lidar parameters
lidar.maxRange    = 10;
lidar.resDeg      = 1;
lidar.startDeg    = -179;
lidar.stopDeg     =  180;
lidar.w           = [0.7 0.2 0.09 0.01];
lidar.sig         = 0.06;
lidar.lambda      = 0.5;
lidar.numScans    = 1+(lidar.stopDeg - lidar.startDeg)/lidar.resDeg;
lidar.forward0    = 0.0; % Laser forward offset (relative to body)
lidar.right0      = 0.0; % Laser right offset (relative to body)
lidar.angle_down0 = 0.0; % Laser down angle offset (relative to body - positive )

