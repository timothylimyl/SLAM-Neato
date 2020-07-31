function new_grid =  bresenhamLineAlgo(grid,pose,xe,ye)

pixeldim = grid.Deast;  % assumption that grid is square

Poccupied   = 0.8;
Punoccupied = 0.65;

ep       = 1 - Punoccupied; % probability of unoccupied when ray passes.

%Loop through all of the points hit by LiDAR available:
for i = 1:length(xe)
    
    %ignore any coordinates that are outside of map.
    if  (xe(i) >  grid.eastSize  || ye(i) > grid.northSize || xe(i) <= 0 || ye(i) <= 0)
        continue;
    end
    
    % Initialise start points
    x(1) = int32(round(pose.east,4)/pixeldim);
    y(1) = int32(round(pose.north,4)/pixeldim);

    % Initialise end points
    x(2) = int32(round(xe(i),4)/pixeldim);
    y(2) = int32(round(ye(i),4)/pixeldim);
    
    % ignore any negative coordinates involved.
    % redundant in this case
    if((x(1)<= 0 || x(2)<= 0 || y(1)<= 0|| y(2)<= 0) )
       continue;
    end
     
    % color the end point (HIT) (OCCUPIED)
    % inverseModel)- at the end of all points, change idx points to 0.
    
    grid.Z(y(2),x(2)) = Poccupied;   
    
    % uncolor point that robot is on (If you are on it, it is unoccupied by
    % obstacles.
    grid.Z(y(1),x(1)) = ep;
    
    
%%%%%%%%%%%%%%%%%% Setup deals with all slopes (360)  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Code was built initially with heuristic used for 0 <= slope < 1

    % if change in y > change in x = slope > 1 or slope < -1
    % swap x and y axis 
    steep = (abs(y(2)-y(1)) > abs(x(2)-x(1)));

    % By swapping the axis, it will work for slope > 1
    % note that all x and y pixels are swapped too.
    if steep
       [x,y] = swap(x,y); % change x=y, y=x
    end

    % We will need to switch the start and ending point if x2 < x1
    % reversing so that the heuristic works the same.
    % 
    if x(2) < x(1)
         [x(1),x(2)] = swap(x(1),x(2));
         [y(1),y(2)] = swap(y(1),y(2)); % switch start and ending point
    end
    % note: reversing also affects quadrant 2

    % Therefore, need to reverse step for y when this happens quadrant 2
    % (negative slope) -step usually +1 , but we need -1 when in quadrant 2.
    % can think of this as the y-axis reflecting on line x-axis to solve for
    % quadrant 2 and 4 from the same heuristic as positive slope.
    if y(1) < y(2)
        ystep = 1;
    else
        ystep = -1;
    end


    %%%%%%%% done and tested - works for  quadrant 1 (0< slope <1) %%%%%%%%%%%%%%

    % Absolute only needed for delta-y cause only y-axis is reflected on
    % x-axis.
    deltay = abs(y(2) - y(1)); % put abs because it is negative for negative slope
    deltax = x(2) - x(1); 
    P = 2*deltay - deltax; %heuristic of distance to next pixel.

    xpixel = x(1);
    ypixel = y(1);
    xstep  = 1;


    while (xpixel < x(2)-1) % Not coloring start and end point.
    % Note that it is < x(2)-1 cause you want the pixel steps to
    % stop a step before it reaches the end point in this case.
    % The end point is colored (OCCUPIED) before this loop runs.


        xpixel = xpixel + xstep;

        if P <= 0

            P = P + 2*deltay;
            %x increment

        else

            P = P + 2*deltay - 2*deltax ;
            %x increment and y
            ypixel = ypixel + ystep;    

        end
        
        % color the next pixel (unoccupied)
        % if slope > 1: need reverse xpixel and ypixel:
        if steep
            grid.Z(xpixel,ypixel) = ep; %color it.
        else
            grid.Z(ypixel,xpixel) = ep; %color it.
        end
        

    end % finish looping till it hits the end point
    
  
end % finish looping through all hit points


% Change all of the max range hits to be undetermined or unoccupied?


new_grid = grid.Z;



end % end function.


            
function [a,b] = swap(x,y)

    a = y; 
    b = x;    

end