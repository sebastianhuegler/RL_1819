function wave = pendulumCode
% A demo of iLQG/DDP with car-parking dynamics
clc;
close all

fprintf(['\nA demonstration of the iLQG algorithm '...
'with pendulum dynamics.\n'...
'for details see\nTassa, Mansard & Todorov, ICRA 2014\n'...
'\"Control-Limited Differential Dynamic Programming\"\n'])

% set up the optimization problem
dt = 0.001;
DYNCST  = @(x,u,i) pen_dyn_cst(x,u,dt);
T       = 5000;              % horizon
x0      = [0;0];   % initial state
u0      = zeros(1,T);    % initial controls
Op.lims = [-4 4];        % control input limits (N/m)
Op.plot = -1;               % plot the derivatives as well

% === run the optimization!
[x,u]= iLQG(DYNCST, x0, u0, Op);

wave.time = [];
wave.signals.values = [u',x(:,1:end-1)'];
wave.signals.dimensions = 3;

% animate the resulting trajectory
figure
for r = 1:10:size(u,2)
     draw_pendulum(x(1,r), u(r), 0,  ...
      ['Optimal control policy, T=' num2str(T*dt) ' sec'], ...
      '')
  pause(dt/10);
end

function y = pendulum_dynamics(x,u,dt)
%% Code

s = 1;    % [m]        length of pendulum
m = 1;    % [kg]       mass of pendulum
g = 9.82; % [m/s^2]    acceleration of gravity
b = 0.2; % [s*Nm/rad] friction coefficient

dx = zeros(2,size(x,2));

dx(1,:) = x(2,:);
dx(2,:) = ( u - b*x(2,:) - m*g*s*sin(x(1,:)) ) / (m*s^2);

y = x + dx*dt;

function c = pendulum_cost(x, u)
% cost function for inverted pendulum problem
% sum of 3 terms:
% lu: quadratic cost on controls
% lf: final cost on distance from target parking configuration
% lx: running cost on distance from origin to encourage tight turns

final = isnan(u(1,:));
u(:,final)  = 0;

cu  = 1e-3;         % control cost coefficients

cf  = [ 100  .1];    % final cost coefficients 10 for enough torque

cx  = 1e-2;          % running cost coefficients 1e-1 for enough torque

% control cost
lu    = cu*u.^2;

% final cost
if any(final)
   llf      = cf*(x(:,final)-[pi;0]).^2;
   lf       = double(final);
   lf(final)= llf;
else
   lf    = 0;
end

% running cost
lx = cx*(x(1,:)-[pi]).^2;

% total cost
c     = lu + lx + lf;

function [f,c,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = pen_dyn_cst(x,u,dt)
% combine car dynamics and cost
% use helper function finite_difference() to compute derivatives

if nargout == 2
    f = pendulum_dynamics(x,u,dt);
    c = pendulum_cost(x,u);
else
    % state and control indices
    ix = 1:2;
    iu = 3;
    
    % dynamics first derivatives
    xu_dyn  = @(xu) pendulum_dynamics(xu(ix,:),xu(iu,:),dt);
    J       = finite_difference(xu_dyn, [x; u]);
    fx      = J(:,ix,:);
    fu      = J(:,iu,:);
    
    [fxx,fxu,fuu] = deal([]);
    
    % cost first derivatives
    xu_cost = @(xu) pendulum_cost(xu(ix,:),xu(iu,:));
    J       = squeeze(finite_difference(xu_cost, [x; u]));
    cx      = J(ix,:);
    cu      = J(iu,:);
    
    % cost second derivatives
    xu_Jcst = @(xu) squeeze(finite_difference(xu_cost, xu));
    JJ      = finite_difference(xu_Jcst, [x; u]);
    JJ      = 0.5*(JJ + permute(JJ,[2 1 3])); %symmetrize
    cxx     = JJ(ix,ix,:);
    cxu     = JJ(ix,iu,:);
    cuu     = JJ(iu,iu,:);
    
    [f,c] = deal([]);
end


function J = finite_difference(fun, x, h)
% simple finite-difference derivatives
% assumes the function fun() is vectorized

if nargin < 3
    h = 2^-17;
end

[n, K]  = size(x);
H       = [zeros(n,1) h*eye(n)];
H       = permute(H, [1 3 2]);
X       = pp(x, H);
X       = reshape(X, n, K*(n+1));
Y       = fun(X);
m       = numel(Y)/(K*(n+1));
Y       = reshape(Y, m, K, n+1);
J       = pp(Y(:,:,2:end), -Y(:,:,1)) / h;
J       = permute(J, [1 3 2]);

% utility functions: singleton-expanded addition and multiplication
function c = pp(a,b)
c = bsxfun(@plus,a,b);