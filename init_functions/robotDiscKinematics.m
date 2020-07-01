function x_tp1 = robotDiscKinematics(x,v,param)

u = (v(1) + v(2))/2;
r = (v(1) - v(2))/param.a;

%x(3,:) = x(3,:) * (pi/180);

x_tp1 = x + [   u.*cos(x(3,:))	;
                u.*sin(x(3,:))	;
                r*ones(1,length(x(1,:))) ]*param.T;
