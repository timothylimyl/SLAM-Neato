function [a,z] = plotInit(grid)
  

   figure('renderer','opengl');
   colormap(flipud(bone)); % flip value of colormap
   % This flips the plot too. 
   % 0 unoccupied, 1 occupied
   % pcolor plots 0 as black, 1 as white, (gets flipped along)

   
   z = grid.Z;
   z = [z zeros(size(z,1),1)];
   z = [z;zeros(1,size(z,2))];

   a = pcolor(grid.east,grid.north,z);
   colorbar
   caxis([0 1])
   daspect([1 1 1]);
   set(a,'edgealpha',0);
   hold on;
   
   plotUpdate(a,z);
   
end