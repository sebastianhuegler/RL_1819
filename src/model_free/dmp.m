function [u,dy] = dmp(z,y,theta,g)
%DMP Summary of this function goes here
%   Detailed explanation goes here
    tau = 1;
    alpha_z =[0.5];
    beta_z = [10];
    alpha_y = 1;
     
    dy = -tau*alpha_y*y;
    % i=1:K;
    mu = linspace(1,0,length(theta));
    sigma = 0.1;
    w = theta;% theta;
    for i = 1:length(theta)
        phi(i,1) = rbf(y,mu(i),sigma); 
    end
    f =w'*phi/sum(phi)*y;% (sum(phi(i))*w)/(sum(phi(i)))*y*(g-y0);
    %u = tau^2*(alpha_z.*beta_z)'*(g(1)-z(1))-tau^2*alpha_z'*z(2)+tau^2*f;
    
    u = tau^2*([alpha_z.*beta_z; alpha_z.*beta_z])'*(g([1 3])-z([1 3]))-tau^2*[alpha_z; alpha_z]'*z([2 4])+tau^2*f;

end

