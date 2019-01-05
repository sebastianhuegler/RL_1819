function [dz] = closed_loop(t,z,theta,number_rbf,g)
%CLOSED_LOOP Summary of this function goes here
%   evaluation of rbf with theta = [w1 ... wn mu11 mu12 ... mun1 mun2,
%   sigma11 sigma12 ... sigman1 sigman2] n is number of rbf

RBF = 0;
%RBF controller
if(RBF)
    for i = 1:number_rbf
        phi(i,:) = rbf(z,theta((number_rbf+length(z)*(i-1))+(1:length(z))),...
            theta(number_rbf*(1+length(z))+i));
        %max(theta((number_rbf*(1+length(z))+2*(i-1))+(1:2)),0.0001*ones(length(z),1)));
    end
    u1 = theta(1:number_rbf)'*phi;
else
    %u1 = [5 5]*(g-z)-[0.1 0.1]*d_z;
    [u1,dy] = dmp(z(1:end-1),z(end),theta,g);
end

% DMP controller
%d_z = dyn_pendulum(t,z,u1);
d_z = dyn_cart_pole(t,z,u1);
%d_z = dyn_mass_spring_damper(t,z(1:end-1),u1);
dz = [d_z; dy];

end

