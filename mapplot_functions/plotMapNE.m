function h = plotMapNE(map)
    z = map.Z;
    z = [z zeros(size(z,1),1)];
    z = [z;zeros(1,size(z,2))];
    h = pcolor(map.east,map.north,1-z);
    set(h,'edgealpha',0.0);
end