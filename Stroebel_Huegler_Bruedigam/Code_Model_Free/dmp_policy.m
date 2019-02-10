function [policy] = dmp_policy(v,theta,settings)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%     alpha_y = settings.alpha_y;
%     beta_y = settings.beta_y;
%     tau = settings.tau;
%     alpha_z = settings.alpha_z;
%     mu = settings.mu;
%     sigma = settings.sigma;
%     g = settings.g;
%
%     % i=1:K;
%
%     w = theta;% theta;
%     phi = zeros(length(theta),1);
%     for i = 1:length(theta)
%         phi(i,1) = rbf(v(3),mu(i),sigma);
%     end
%     f =w'*phi/sum(phi).*v(:,3);% (sum(phi(i))*w)/(sum(phi(i)))*y*(g-y0);
%
%     dy2 = tau^2*alpha_y*beta_y.*(g-v(:,1))-tau*alpha_y.*v(:,2)+tau^2*f;
%     policy = dy2;
alpha_y = settings.alpha_y;
beta_y = settings.beta_y;
tau = settings.tau;
alpha_z = settings.alpha_z;
mu = settings.mu;
sigma = settings.sigma;
g = settings.g;
number_rbf = settings.number_rbf;
maxU = settings.maxU;

dz = -tau*alpha_z*v(length(g)+1);
% i=1:K;

w = theta(1:(number_rbf/length(g)));% theta;
phi = zeros(number_rbf/length(g),1,length(v(:,1)));


for j = 1:length(g)
    for k = 1:length(v(:,1))
        for i = 1:(number_rbf/length(g))
            phi(i,j,:) = rbf(v(:,2*length(g)+1),mu(i),sigma(i));
        end
        
        f(k,j) =w'*phi(:,j,k)/sum(phi(:,j,k))*v(k,2*length(g)+1);% (sum(phi(i))*w)/(sum(phi(i)))*y*(g-y0);
    end
   % dy(:,1+(2*(j-1))) = v(:,2+(2*(j-1)));
    dy(:,2+(2*(j-1))) = min(max(tau^2*alpha_y*beta_y*(g(j)-v(:,1+(2*(j-1))))-tau*alpha_y*v(:,2+(2*(j-1)))+tau^2*f(:,j),-maxU),maxU);
end
policy = dy(:,2);
end

