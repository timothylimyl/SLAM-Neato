function lidarNoisyScan0 = simulateNoisyLidarScan(map,lidar,pose)

pose.psi =  pose.psi * (180/pi);

[zh,~,~] = scanNE(map,lidar,pose);

% trueLambda = 0.5;
trueSigma  = 0.02;

lidarNoisyScan0 = zeros(size(zh));

randChance = rand(1,lidar.numScans)*100;
for i=1:lidar.numScans
    % 2/100 chance to read max range
    if randChance(i) <= 2
        lidarNoisyScan0(i) = lidar.maxRange;
        
    % 1/100 chance to read random range
    elseif randChance(i) <= 3
        lidarNoisyScan0(i) = lidarNoisyScan0(i) + 0.01*randn;
        
    % 5/100 chance to read short range
    elseif randChance(i) <= 6
        lidarNoisyScan0(i) = rand*zh(i);
        
    % 90/100 chance to read random gaussian range near true hit
    else
        lidarNoisyScan0(i) = zh(i) + trueSigma*randn;
        
    end
end
