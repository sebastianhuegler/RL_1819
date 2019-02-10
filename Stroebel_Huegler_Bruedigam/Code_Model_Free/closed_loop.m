function [dz] = closed_loop(t,z,policy,time,mdl,control_mode)
%CLOSED_LOOP Summary of this function goes here
%   evaluation of rbf with theta = [w1 ... wn mu11 mu12 ... mun1 mun2,
%   sigma11 sigma12 ... sigman1 sigman2] n is number of rbf

K_p = [10; 0];%10*ones(max(length(z)/2,1),1);
K_d = [1; 0];%10*ones(max(length(z)/2,1),1);

% without controller
if(control_mode == 0)
    u1 = interp1(time',policy,t);
elseif(control_mode == 1)
    % with controller
    u1 = K_p'*(interp1(time',policy(:,1:2:end-1),t)'-z(1:2:end)) +K_d'*(interp1(time',policy(:,2:2:end-1),t)'-z(2:2:end))+interp1(time',policy(:,end),t);
end

if(mdl == 1)
    d_z = dyn_mass_spring_damper(t,z,u1);
elseif(mdl == 2)
    d_z = dyn_pendulum(t,z,u1);
elseif(mdl == 3)
    d_z = dyn_cart_pole(t,z,u1);
end
dz = [d_z];

end

