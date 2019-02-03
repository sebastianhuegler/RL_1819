function dz = dyn_mass_spring_damper(t,z,u)
%DYN_MASS_SPRING_DAMPER Summary of this function goes here
%   Detailed explanation goes here
c = 1; %[N/m]
d = 0; %[Ns/m]
m = 1; %[kg]
% A = zeros(4,4);
% B = zeros(4,4);
A(1:2,1:2) = [0 1; -c/m -d/m];
B(1:2,1:2) = [0 0; 0 1/m];
dz = A*z+B*[0; u(t)];
end