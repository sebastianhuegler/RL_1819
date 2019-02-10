%% Settings
clear all
close all

%number_rbf = 20;
dt = 0.025;
T = 2.5;
K = 10000; %epochs
OPTIONS = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
mdl = 1; % 1 - spring damper mass, 2 - pendulum, 3 - cartpole
control_mode = 0; % 1 - use PD control to follow DMP, 0 - uncontrolled policy
dynamics_mdl = @closed_loop;

% reward/exploration settings
sigma_reward = 1/sqrt(2*pi);
alpha = 1/(2*pi*sigma_reward^2);
sigma = 5; %CARTPOLE: 1; PENDULUM: 5
R_importance(1,:) = [0 0];
idx = 0; % initialize index for 10 best PoWER parameters
% tolerance = 0.05; %m
reward = @(x,x_target,a) exp(-a*diag((x-x_target)*(x-x_target)'));
% reward = @(x,x_target,a) max(zeros(length(x),1),1-(a*diag((x-x_target)*(x-x_target)')));
%reward = @(x,x_target) exp((x(end)-x_target)'*(x(end)-x_target))+ sum(exp(-diag((x-x_target)'*(x-x_target)))/length(x));




% Initialize variables for states parameters
if(mdl == 1)
    x_target = [1.5;0]; % spring damper mass
    x_initial = [0;0];  %spring damper mass
    reward_states = [1 2];
    
elseif (mdl == 2)
    x_target = [(pi);0];% pendulum
    x_initial = [0;0];  % pendulum
    reward_states = [1 2];
    
elseif (mdl == 3)
    x_target =  [0; 0;  (pi);    0];    % cart pole
    x_initial = [0; 0;  180/180*pi; 0];  %cart pole: [0;0;x/180*pi;0] --> x = 10°, 90°, 180°
    reward_states = [1 2 3 4];
    alpha = [1 0.1 1 0.1];
    reward = @(x,x_target,a) exp(-diag((a.*(x-x_target))*(x-x_target)'));
    
else
    error('model not available')
end




%dmp settings
dmp_settings.g = x_target(1);%x_target(1:2:(end-1));
dmp_settings.tau = 1; %Skalierungskonstante: bestimmt, über welche Zeit die DMP ablaufen soll
dmp_settings.alpha_y =[10]; %pendulum = 5 cart-pole = 10
dmp_settings.beta_y = [1]; %pendulum = 1 cart-pole = 10
dmp_settings.alpha_z = 0.7;
dmp_settings.number_rbf = 40; %CARTPLOLE: 80, PENDULUM: 40
dmp_settings.mu = exp(-linspace(0,1,(dmp_settings.number_rbf)));
dmp_settings.sigma = 0.2*sqrt(dmp_settings.mu/dmp_settings.number_rbf);%0.02*ones(size(dmp_settings.mu));%*sqrt(dmp_settings.mu/dmp_settings.number_rbf);%0.04;
dmp_settings.init_state = [zeros(length(dmp_settings.g)*2,1);1];
dmp_settings.maxU = 10;

% Initialize parameter vector;
%theta_rbf(:,1) = zeros(number_rbf*(1+length(x_target)+1),1);
% theta for DMP
theta_dmp(:,1) = zeros(dmp_settings.number_rbf*length(dmp_settings.g),1);
theta = theta_dmp;

%[time, x] = ode45(dynamics_mdl,[0:dt:T],[0;0]);
% for i = 1:number_rbf
%     phi(i,:) = rbf(x,theta(number_rbf+(i:i+1)),theta(number_rbf*(1+length(x_target))+(i:i+1)));
% end
%
% [time, x] = ode45(dynamics_mdl,[0:dt:T],[0;0],OPTIONS,time,u')
%% PoWER-Learning Algorithmus
% s_present = S(3,1); %first initial state
%x = zeros(length([0:dt:T]),2);
time = [0:dt:T];

figure
for k = 1:K %each epoch
    %% Exploration of policy
    % randomly initialize parametervector
    theta_temp(:,k) = theta(:,k) + sigma*randn(length(theta(:,k)),1); %max((1-sum(R_importance(max(idx,1),1))/max(idx+1e-2)),0.01)*
    
    % Simulate DMP
    [y] = ode3(@dmp,time,dmp_settings.init_state,theta_temp(:,k),dmp_settings);  %time: simulationszeit
    
    if(control_mode == 0)
        %without controller
        policy = dmp_policy(y,theta_temp(:,k),dmp_settings);%y(:,1:2);%
    elseif(control_mode == 1)
        %with controller
        policy = [y(:,1:end-1) dmp_policy(y,theta_temp(:,k),dmp_settings);];%[x_target(1)*ones(length(y(:,1)),1),zeros(length(y(:,1)),1),zeros(length(y(:,1)),1)];%
    end
    %[time, rollout] = ode45(dynamics_mdl,[0:dt:T],x_initial,OPTIONS,theta_temp(:,k),number_rbf);
    [rollout] = ode3(dynamics_mdl,time,x_initial,policy,time,mdl,control_mode);
    rollout = rollout(:,1:length(x_target));
    % wrap angle of models with 2*pi
    if(mdl == 2)
        rollout(:,1) =  mod(rollout(:,1),2*pi);% cos(rollout(:,1));%     %PENDULUM
    elseif(mdl == 3)
        rollout(:,3) = mod(rollout(:,3),2*pi);%cos(rollout(:,3));%mod(rollout(:,3),2*pi); %CARTPOLE
    end
    
    if(mod(k,100)==0)
        subplot(3,1,1)
        plot(time,rollout)
        hleg1 = legend('x','$\dot{x}$','$\theta$','$\dot{\theta}$');
        set(hleg1,'interpreter','latex')
        subplot(3,1,2)
        plot(R_importance(:,1))
        subplot(3,1,3)
        plot(R_importance(:,2),R_importance(:,1),'.')
        refreshdata
        pause(0.1)
        disp(k)
        
    end
    
    %% Evaluation of policy
    % calculate reward
    R(k) = sum(reward(rollout(:,reward_states),x_target(reward_states)',alpha))/length(rollout); %rollout(end,:),x_target',alpha)+
    R_importance(k,:) = [R(k) k];
    R_importance = sortrows(R_importance,'descend');
    
    %% Update policy by PoWER
    %     for i = 1:min(length(R_importance(:,1)),10)
    %         delta_theta(:,i) = theta_temp(:,R_importance(i,2))-theta(:,k);
    %     end
    if(k>10)
    idx = 1:min(length(R_importance(:,1)),10);
    delta_theta(:,idx) = ...
        theta_temp(:,R_importance(idx,2))-theta(:,k);
    
    theta(:,k+1) = theta(:,k) + sum(delta_theta.*R_importance(idx,1)',2)./sum(R_importance(idx,1));
    else
        theta(:,k+1) = zeros(size(theta_temp(:,1)));
    end
    %theta_temp(:,R_importance(1,2));%
    
    
end
% Simulate DMP
[y] = ode3(@dmp,time,dmp_settings.init_state,theta(:,end),dmp_settings);  %time: simulationszeit
if(control_mode == 0)
    %without controller
    policy = dmp_policy(y,theta(:,k),dmp_settings);%y(:,1:2);%
elseif(control_mode == 1)
    %with controller
    policy = [y(:,1:end-1) dmp_policy(y,theta(:,k),dmp_settings);];%[x_target(1)*ones(length(y(:,1)),1),zeros(length(y(:,1)),1),zeros(length(y(:,1)),1)];%
end
final_rollout = ode3(dynamics_mdl,time,x_initial,policy,time,mdl,control_mode);
figure
plot(time,final_rollout)
hleg = legend('x','$\dot{x}$','$\theta$','$\dot{\theta}$')
set(hleg,'interpreter','latex')
disp('habe fertig')