function hh = readyFigure(map)

hmap = figure(1);
set(hmap,'renderer','opengl')
colormap('bone'); 
plotMapNE(map);
daspect([1 1 1])
hold on;

hs=[];
ht=[];
rh=[];

hh = {hmap,hs,ht,rh};