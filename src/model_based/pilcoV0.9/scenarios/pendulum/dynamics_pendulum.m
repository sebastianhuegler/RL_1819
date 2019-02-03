function dz = dynamics_pendulum(t,z,u)
%PENDULUM_DYN describes the discrete time dynamics of a pendulum
%   Detailed explanation goes here
m = 1; % kg
l = 1; % m
c = 0.2; % Ns/m
g = 9.81; %

dz(2,:) =  z(1);
dz(1,:) = (u(t)/(m*l^2) - g/l*sin(z(2))-c/(m*l^2)*z(1));


end


