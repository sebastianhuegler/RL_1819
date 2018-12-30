s = 1;    % [m]        length of pendulum
m = 1;    % [kg]       mass of pendulum
g = 9.82; % [m/s^2]    acceleration of gravity
c = 0.2; % [s*Nm/rad] friction coefficient

A = [
    0 1
    g -c
    ];

B = [
    0
    1
    ];

C = [1 0];

D = 0;
%%%

%Weights
Q = [
    1 0
    0 1
    ];

R = .1;

%K
[K,S,e] = lqr(A,B,Q,R);

%prefilter matrix
L = inv(C/(B*K-A)*B);

sys=ss(A,B,C,D);
sysIO = ss(tf([1 1],[1])*tf(sys));
