function [muy,Py] = UnscentedTransformUpdate(grid,laser,states,mu,P)


%First generate the sigma points
n = size(P,1);
sP = chol(P).';
p = sqrt(n)*sP;
x = mu + p;
x = [x mu-p];


%Propagate through the nonlinearity
for i=1:2*n
    
    y(:,i) = update(x(:,i),grid,laser,states);
    
    
end



%Generate the sample mean and covariance
muy = sum(y,2)/(2*n);
Py = (y - muy)*(y - muy).'/(2*n);