function [updateUKF]  = update(x,grid,laser,n)
   
   pose.north = x(1);
   pose.east  = x(2);
   pose.psi   = x(3) * (180/pi);
   
   % Lidar Ranges (Raycast on new map and pose):
   
   [lidar_ranges,~,~] = scanNE(grid,laser,pose);

    
   updateUKF = [x(1:n);
               lidar_ranges + x(n+1:end)];
          


         
end
