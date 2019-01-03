function dz = dynamics_smd(z, f)
    % Parameters
    m = 1;  % [kg]     mass
    k = 1;  % [N/m]     spring constant
    c = 0.0;  % [Ns/m]  damping coefficient

    dz = zeros(2,1);

    % state [x, xd]
    dz(1) = z(2);
    dz(2) = (f - c*z(2) - k*z(1))/m;
end