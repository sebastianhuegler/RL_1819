function dz = dyn_mass_spring_damper(t,z,u1)
%DYN_MASS_SPRING_DAMPER Summary of this function goes here
%   Detailed explanation goes here
c = 1; %[N/m]
d = 0; %[Ns/m]
m = 1; %[kg]
A = [0 1; -c/m -d/m];
B = [0 0; 0 1/m];
% if(exist('u'))
%     u1 = interp1(ut,u,t);
% else
%     for i = 1:number_rbf
%         phi(i,:) = rbf(z,theta((number_rbf+2*(i-1))+(1:2)),...
%             max(theta((number_rbf*(1+length(z))+2*(i-1))+(1:2)),zeros(length(z),1)));
%     end
%     
%     u1 = theta(1:number_rbf)'*phi;
% end
dz = A*z+B*[0; u1];
end