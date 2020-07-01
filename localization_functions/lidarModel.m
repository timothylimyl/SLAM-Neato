function ll  = lidarModel(measurements,grid,laser,pose)
    
    pose.psi  = pose.psi*(180/pi); % convert to degrees for scanNE
    
    % Preventing using scanNE when particles are outside of map. (cannot
    % exit mex function)
    if  pose.east <=  grid.eastSize  &&  pose.north <= grid.northSize && pose.east > 1 && pose.north > 1

        lidarMeasurement = measurements;
        
        w = laser.w;
        whit   = w(1); %hit
        wshort = w(2); %early obstacle (unexpected object)
        wmax   = w(3); %maximum reading
        wrand  = w(4); %random object
        
        sigma  = laser.sig;
        lambda = laser.lambda;

        %implements ray casting:
        [lidar_ranges,~,~] = scanNE(grid,laser,pose);
        % lidar_ranges == Ray Casting (yhat)
        % lidarRanges  == Actual Measurements
  
        
        % integration to find normalising constant  %%%%%%%%%%%%%%%%%%%%%%%
        % fun    = @(x) exp( (-( x - lidar_ranges(1:length(lidar_ranges)) ).^2) ./ (2.*sigma.^2));
        % q      = integral(fun,0,laser.maxRange,'ArrayValued',true);
        % To save time, we instead find the expression at the start (command window), copy paste expression:
        % syms x y sigma
        % fun    = @(x) exp( (-( x - y )^2) / (2*sigma^2));
        % int(fun,x,0,laser.maxRange)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Likelihood of hitting expected map obstacle:
        q = -(2^(1/2)*pi^(1/2)*(erf((2^(1/2).*(1/sigma^2).^(1/2).*(lidar_ranges - 120))/2) - erf((2^(1/2).*lidar_ranges .*(1/sigma^2)^(1/2))/2)))./(2*(1/sigma^2)^(1/2));
        cnst   = (1/sqrt(2*pi*sigma^2)) .* q ;
        phit   = (1./cnst) .* (1./(sqrt(2*pi*sigma^2))) .*  exp( (-1/(2*sigma^2)) .* (lidarMeasurement - lidar_ranges).^2  ) .* double( lidarMeasurement <= laser.maxRange-1);
        
        % Likelihood of detecting unexpected near obstacle:
        pshort = zeros(length(lidarMeasurement),1);
        s_idx  = lidarMeasurement <= lidar_ranges ;
        pshort(s_idx) = (1./(1-exp(-lambda.*lidar_ranges(s_idx)))) .*  (lambda.*exp(-lambda.*lidarMeasurement(s_idx)));
        
        % Likelihood of sensor failure (assume lidar gives back max range)
        pmax   = double(lidarMeasurement == laser.maxRange); 
        
        % Likelihood of random background noise
        prob_rand  = 1/laser.maxRange;
        prand = prob_rand .* double(lidarMeasurement <= laser.maxRange-1);        % prand has been pre-initialise with zeros
        
        % Mixture Distribution:
        prob = whit.*(phit) + wshort.*(pshort) + wmax.*(pmax) + wrand.*(prand); 
        ll = sum(log(prob)); % sum all the lidar scans
        
    else %high unlikely that it is outside of map:
        ll = -Inf;
    end

end