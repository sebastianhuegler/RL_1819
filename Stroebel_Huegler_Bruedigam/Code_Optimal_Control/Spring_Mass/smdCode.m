clear all

m = 1;  % [kg]     mass
k = 1;  % [N/m]     spring constant
c = 0.0;  % [Ns/m]  damping coefficient

A = [
    0    1
    -k/m 0
    ];

B = [
    0
    1/m
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

%Noise
N = eye(2);

%Complete System
Ak = A - B*K;
Bk = B*L;
sys = ss(Ak,Bk,C,D);

%System with PI I/O controller 
sysIO = ss(tf([1 1],[1 0])*tf(sys));
%sysIO = ss(tf([0 1],[1 0])*tf(sys));
