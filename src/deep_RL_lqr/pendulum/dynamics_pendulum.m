function dz = dynamics_pendulum(z,f)
%% Code

s = 1;    % [m]        length of pendulum
m = 1;    % [kg]       mass of pendulum
g = 9.82; % [m/s^2]    acceleration of gravity
c = 0.2; % [s*Nm/rad] friction coefficient

dz = zeros(2);

dz(1) = z(2);
dz(2) = ( f - c*z(2) - m*g*s*sin(z(1)) ) / (m*s^2);

end