% Initialise parameters for empty occupancy grid cells and padded map for
% plotting
function grid = initOccupancyGrid()

initial_probability = 0.5;

grid.Z   = loadMapFromImage('mapImage.png')*0 + initial_probability;
grid.Neast  = size(grid.Z,2);  %Number of East-axis points
grid.Nnorth = size(grid.Z,1);  %Number of North-axis points
grid.Deast  = 0.02;          % 0.2 East-axis cell dimension (in metres)
grid.Dnorth = 0.02;          % 0.2 North-axis cell dimension (in metres)
grid.east0  = 0;            %Origin offset East-axis
grid.north0 = 0;            %Origin offset North-axis

% individual cell occupancy
grid.east   = grid.east0:grid.Deast:(grid.east0+grid.Deast*(grid.Neast));
grid.north  = grid.north0:grid.Dnorth:(grid.north0+grid.Dnorth*(grid.Nnorth));

% Size of map (To make particles out of map to be -Inf log weights)
grid.eastSize = grid.Neast * grid.Deast;
grid.northSize = grid.Nnorth * grid.Dnorth;


end
