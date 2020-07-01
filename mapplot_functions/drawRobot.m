function rh = drawRobot(x)

% N   = pose.north;
% E   = pose.east;
% psi = pose.psi;

N   = x(1,end);
E   = x(2,end);
psi = x(3,end);

% robot dimensions
rr = 0.1; % robot radius (10 cm)

% Draw robot as a circle with forward and right axis
rh(1) = rectangle('Position',[E-rr, N-rr, 2*rr, 2*rr], 'Curvature', [1 1], 'EdgeColor', 'r',  'FaceColor', [1 0.7 0.7]);
set(rh(1),'linewidth',1)
rh(2)=line([E E+rr*1.5*sin(psi)],[N N+rr*1.5*cos(psi)]);
set(rh(2),'linewidth',1,'color','b');
rh(3)=line([E E+rr*sin(psi+pi/2)],[N N+rr*cos(psi+pi/2)]);
set(rh(3),'linewidth',1,'color','b');
rh(4)=plot(x(2,:),x(1,:));
set(rh(4),'linewidth',1,'color','g');
drawnow;
