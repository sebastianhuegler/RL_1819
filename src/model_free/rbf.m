function [value] = rbf(x,mu,sigma)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
if size(x,1) == size(mu,1) && size(x,2) == size(mu,2)
    pow = -0.5*(x-mu)'*(x-mu)/sigma^2;%-0.5*(x-mu)'*diag(di)*(x-mu);
    value = exp(pow);
elseif size(x,1)>size(x,2)
    value = exp(-0.5*diag((x-mu')*(x-mu')')/sigma^2);
else
        error('rbf not working')
end

end

