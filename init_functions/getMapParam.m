function map = getMapParam()

map.Z   = loadMapFromImage('mapImage.png');

% Generate map map
map.Neast  = size(map.Z,2);     % Number of East-axis points
map.Nnorth = size(map.Z,1);     % Number of North-axis points
map.Deast  = 0.02;              % East-axis cell dimension (in metres)
map.Dnorth = 0.02;              % North-axis cell dimension (in metres)
map.east0  = 0.0;               % Origin offset East-axis
map.north0 = 0.0;               % Origin offset North-axis

% Initialise other map variables
% map.Z      = 0.5*ones(map.Nnorth,map.Neast); % pad with 0.5 for initial map
map.east   = map.east0:map.Deast:(map.east0+map.Deast*(map.Neast));
map.north  = map.north0:map.Dnorth:(map.north0+map.Dnorth*(map.Nnorth));



map.eastSize = map.Neast * map.Deast;
map.northSize = map.Nnorth * map.Dnorth;