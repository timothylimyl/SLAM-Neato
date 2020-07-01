function [muy,Py] = UnscentedTransform(fcn,mu,P)
    %First generate the sigma points
    n   = size(P,1);
    sP  = chol(P).';
    p   = sqrt(n)*sP;
    x   = [mu+p mu-p];

    %Propagate through the nonlinearity
    y   = zeros(size(x,1),2*n);
    for i = 1:2*n
        y(:,i) = fcn(x(:,i));
    end

    %Generate the sample mean and covariance
    muy = sum(y,2)/(2*n);
    Py  = (y - muy)*(y - muy).'/(2*n);
end