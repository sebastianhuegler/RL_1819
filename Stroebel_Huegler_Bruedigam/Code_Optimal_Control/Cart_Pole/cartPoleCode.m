%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MPC for the cart-pole system
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function log =  cartPoleCode
    global testLog;
    testLog = zeros(5,1);

    addpath('./nmpcroutine');
    mpciterations = 250;    % Iterations
    N             = 20;     % Horizon
    T             = 0.02;   % Sample time
    tmeasure      = 0.0;    % Initial time
    xmeasure      = [0 0 pi/2/9 0]; % Initial position
    u0            = zeros(1,N);     % Initial control sequence
    
    % Optimization Parameters
    tol_opt       = 1e-8;
    opt_option    = 0;
    iprint        = 5;
    type          = 'difference equation';
    atol_ode_real = 1e-12;
    rtol_ode_real = 1e-12;
    atol_ode_sim  = 1e-4;
    rtol_ode_sim  = 1e-4;

    [t, x, u] = nmpc(@runningcosts, @terminalcosts, @constraints, ...
         @terminalconstraints, @linearconstraints, @system, ...
         mpciterations, N, T, tmeasure, xmeasure, u0, ...
         tol_opt, opt_option, ...
         type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
         iprint, @printHeader, @printClosedloopData, @plotTrajectories);
    rmpath('./nmpcroutine');
    
    log=testLog;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definition of the NMPC functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function cost = runningcosts(t, x, u)
    Q = [
        20 0 0 0
        0 10 0 0
        0 0 10 0
        0 0 0 0
        ];
    R = 0.01;
    x(:,3)=angleInterp(x(:,3));
    xd = (x - [0 0 pi 0]);
    cost = 0.5*(xd*Q*(xd') + u^2*R);
end

function cost = terminalcosts(t, x)
    Q = diag([0 10 10 10]);
    x(:,3)=angleInterp(x(:,3));
    xd = (x-[0 0 pi 0]);
    cost = 0.5*xd*Q*(xd');
end

function theta = angleInterp(theta)
    theta = abs(theta);
end

function [c,ceq] = constraints(t, x, u)
     c(1)= x(1)-4;
     c(2)= -x(1)-4;
     c(3)= u(1)-5;
     c(4)= -u(1)-5;
    
    %c = [];
    ceq = [];
end

function [c,ceq] = terminalconstraints(t, x)
    c   = [];
    ceq = [];
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb  = [];
    ub  = [];
end

function y = system(t, x, u, T)
    l = 0.5;  % [m]      length of pendulum
    m = 0.2;  % [kg]     mass of pendulum
    M = 1;  % [kg]     mass of cart
    c = 0.2;  % [N/m/s]  coefficient of friction between cart and ground
    g = 9.82; % [m/s^2]  acceleration of gravity

    dx = zeros(1,4);
    
    % Cart state [x, xd]
    dx(1) = x(2);
    dx(2) = ( 2*m*l*x(4)^2*sin(x(3)) + 3*m*g*sin(x(3))*cos(x(3)) ...
          + 4*u(1) - 4*c*x(2) )/( 4*(M+m)-3*m*cos(x(3))^2 );

    % Pole state [theta, dtheta]
    dx(3) = x(4);% +(rand-.5)/100;
    dx(4) = (-3*m*l*x(4)^2*sin(x(3))*cos(x(3)) - 6*(M+m)*g*sin(x(3)) ...
          - 6*(u(1)-c*x(2))*cos(x(3)) )/( 4*l*(m+M)-3*m*l*cos(x(3))^2 );


    y = x + dx*T;   
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot resulting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function printHeader()
    fprintf('   k  |      u(k)        x(1)        x(2)        x(3)        x(4)        Cost     Time\n');
    fprintf('--------------------------------------------------------------------------------------\n');
end

function printClosedloopData(mpciter, u, x, t_Elapsed, cost)
    fprintf(' %3d  | %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+6.3f  %+6.3f', mpciter, ...
            u(1,1), x(1), x(2), x(3), x(4), cost, t_Elapsed);
end

function plotTrajectories(dynamic, system, T, t0, x0, u, ...
                          atol_ode, rtol_ode, type, testLog)
    [x, t_intermediate, x_intermediate] = dynamic(system, T, t0, ...
                                          x0, u, atol_ode, rtol_ode, type);
    global testLog                                  
    testLog(1,end+1)=u(1);
    testLog(2:5,end)=x_intermediate(1,1:4)';
    
    figure(1)
    subplot(3,1,1)
    plot(u,'b','linewidth',2);
    subplot(3,1,2)
    hold on
    plot(t_intermediate,x_intermediate(:,1),'b','linewidth',2);
    plot(t_intermediate,x_intermediate(:,2),'r','linewidth',2);
    subplot(3,1,3)
    hold on
    plot(t_intermediate,x_intermediate(:,3),'b','linewidth',2);
    plot(t_intermediate,x_intermediate(:,4),'r','linewidth',2);
    
    % Parameters
    l = 0.5;
    xmin = -3; 
    xmax = 3;    
    height = 0.1;
    width  = 0.3;
    maxU = 10;

    % Compute positions 
    cart = [ x_intermediate(1,1) + width,  height
             x_intermediate(1,1) + width, -height
             x_intermediate(1,1) - width, -height
             x_intermediate(1,1) - width,  height
             x_intermediate(1,1) + width,  height ];
    pendulum = [x_intermediate(1,1), 0; x_intermediate(1,1)+2*l*sin(x_intermediate(1,3)), -cos(x_intermediate(1,3))*2*l];


    figure(2)
    clf;hold on
    plot(0,2*l,'k+','MarkerSize',20,'linewidth',2)
    plot([xmin, xmax], [-height-0.03, -height-0.03],'k','linewidth',2)

    % Plot force
    plot([0 u(1)/maxU*xmax],[-0.3, -0.3],'g','linewidth',10)

    % Plot the cart-pole
    fill(cart(:,1), cart(:,2),'k','edgecolor','k');
    plot(pendulum(:,1), pendulum(:,2),'r','linewidth',4)

    % Plot the joint and the tip
    plot(x_intermediate(1,1),0,'y.','markersize',24)
    plot(pendulum(2,1),pendulum(2,2),'y.','markersize',24)

    % Text
    text(0,-0.3,'applied force')
    set(gca,'DataAspectRatio',[1 1 1],'XLim',[xmin xmax],'YLim',[-1.4 1.4]);
    axis off;
    drawnow;
    

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
