function dz = dyn_pendulum(t,z,u)
%PENDULUM_DYN describes the discrete time dynamics of a pendulum
%   Detailed explanation goes here
m = 1; % kg
l = 1; % m
c = 0.2; % Ns/m
g = 9.81; %

dz(1,:) =  z(2);
dz(2,:) = (u/(m*l^2) - g/l*sin(z(1))-c/(m*l^2)*z(2));


end



