function zp = discrete_cart_pole(z,f,dt)
% Parameters
l = 0.5;  % [m]      length of pendulum
m = 0.2;  % [kg]     mass of pendulum
M = 1;  % [kg]     mass of cart
c = 0.2;  % [N/m/s]  coefficient of friction between cart and ground
g = 9.82; % [m/s^2]  acceleration of gravity

dz = zeros(4,1);
zp = zeros(4,1);

% Cart state [x, xd]
dz(1) = z(2);
dz(2) = ( 2*m*l*z(4)^2*sin(z(3)) + 3*m*g*sin(z(3))*cos(z(3)) ...
      + 4*f - 4*c*z(2) )/( 4*(M+m)-3*m*cos(z(3))^2 );
  
% Pole state [theta, dtheta]
dz(3) = z(4);
dz(4) = (-3*m*l*z(4)^2*sin(z(3))*cos(z(3)) - 6*(M+m)*g*sin(z(3)) ...
      - 6*(f-c*z(2))*cos(z(3)) )/( 4*l*(m+M)-3*m*l*cos(z(3))^2 );

  
zp = z + dz*dt;   
end