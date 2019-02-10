function [dv] = dmp(t,v,theta,settings)
%DMP Summary of this function goes here
%   Detailed explanation goes here
alpha_y = settings.alpha_y;
beta_y = settings.beta_y;
tau = settings.tau;
alpha_z = settings.alpha_z;
mu = settings.mu;
sigma = settings.sigma;
g = settings.g;
number_rbf = settings.number_rbf;

dz = -tau*alpha_z*v(2*length(g)+1);
% i=1:K;

w = theta(1:number_rbf/length(g));% theta;
phi = zeros(number_rbf/length(g),1);


for j = 1:length(g)
    for i = 1:(number_rbf/length(g))
        phi(i,j) = rbf(v(2*length(g)+1),mu(i),sigma(i));
    end
    
    f(j) =w'*phi(:,j)/sum(phi(:,j))*v(2*length(g)+1);% (sum(phi(i))*w)/(sum(phi(i)))*y*(g-y0);
    
    dy(1+(2*(j-1))) = v(2+(2*(j-1)));
    dy(2+(2*(j-1))) = tau^2*alpha_y*beta_y*(g(j)-v(1+(2*(j-1))))-tau*alpha_y*v(2+(2*(j-1)))+tau^2*f(j);
end
dv = [dy';dz];
end

