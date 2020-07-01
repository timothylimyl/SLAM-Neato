% Function to make a fair distribution of particles over map
% Very useful for global localisation or initialisation for localisation.

function x  = globalParticles(x,grid,number_of_particles)


M = number_of_particles;



x(1,:,1) = (grid.northSize) * rand(M,1) + grid.Dnorth; %+size dim to prevent 0 
x(2,:,1) = (grid.eastSize) * rand(M,1) + grid.Deast;  
% Good idea but to make it even better, ignore all of the occupied cells.

map = grid.Z;
% Finding free space for robot
[row,column]= find(~map); % finding free space indices
% occupancy grid location of free spaces
row = row.*grid.Dnorth;
column = column.*grid.Deast;



for i = 1:M
    
    % If particles fall into an occupied cell: change it to an unoccupied
    % cell (shifting particles away from occupied to unoccupied)
    if map(int32(floor(x(1,i,1)/grid.Dnorth)),int32(floor(x(2,i,1)/grid.Deast))) == 1
        
        j = int32(length(row) * rand  ); % selection of free space
        if j == 0
            j=1;
        end
        x(1,i,1) = row(j);
        x(2,i,1) = column(j);
        
        
    end
    
    
end

% angle need bigger intervals, example 1 to 3, no need anything in between:
x(3,:,1) = 359*(pi/180) * rand(M,1);

