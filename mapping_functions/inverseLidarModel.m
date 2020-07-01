function  [xe,ye] = inverseLidarModel(pose,laser,r)
        
        pose.psi = pose.psi * (180/pi);
        
        n   = laser.numScans;
        idx = 0:n-1;

        x0  = pose.east  - laser.right0;
        y0  = pose.north - laser.forward0;
        h0  = pose.psi   - laser.angle_down0;
        
%       xe  = x0 + r.*sin((h0 + laser.startDeg + laser.resDeg*idx(:))*pi/180);
%       ye  = y0 + r.*cos((h0 + laser.startDeg + laser.resDeg*idx(:))*pi/180);        
        

        % filter out max range within 1m of it:
        % max range could be due to shortcomings of lidar rays
        % it is a safe and better decision to not include it at all in our
        % readings as using this reading is added risk to our system.
        % Example: Detecting that the ray path is unoccupied but it is 
        %          actually a sensor failure may cause obstacle colision.
        
        index = r < laser.maxRange ;
        xe    = x0 + r(index).*sin((h0 + laser.startDeg + laser.resDeg.*idx(index)').*pi/180);
        ye    = y0 + r(index).*cos((h0 + laser.startDeg + laser.resDeg.*idx(index)').*pi/180);        
       
        % Filter out off grid points:
        
        %xe= xe.*double(xe > 0);
        %ye= ye.*double(ye > 0); 
        
        %dbstop in inverseLidarmodel at 16 if min(ye)<=0
        %dbstop in inverseLidarmodel at 16 if min(xe)<=0

end