%% Settings
clear all
close all

number_rbf = 20;
dt = 0.05;
T = 4;
K = 10000; %epochs
OPTIONS = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
dynamics_mdl = @closed_loop;

% reward/exploration settings
alpha = 10;
sigma = 1;
R_importance(1,:) = [0 0];
idx = 0;
% tolerance = 0.05; %m
reward = @(x,x_target,a) exp(-a*diag((x-x_target)*(x-x_target)'));
% reward = @(x,x_target,a) max(zeros(length(x),1),1-(a*diag((x-x_target)*(x-x_target)')));
%reward = @(x,x_target) exp((x(end)-x_target)'*(x(end)-x_target))+ sum(exp(-diag((x-x_target)'*(x-x_target)))/length(x));



% Initialize vairbales for states parameters
x_target_rbf= [cos(pi);0];%[0;0;cos(pi);0]; %[pi; 0]; %m m/s
x_initial_rbf = [0;0];%[0;0];
x_target_dmp = [0;0;pi;0];% [1.5;0; 0];
x_initial_dmp = [0;0;0;0;1];
x_target = x_target_dmp;
x_initial = x_initial_dmp;

% Initialize parameter vector;
% theta = [w1 ... wn, [mu11 mu12] ... [mun1 mun2], sigma1 sigman]'; for RBF
theta_rbf(:,1) = zeros(number_rbf*(1+length(x_target)+1),1);
% theta for DMP
theta_dmp(:,1) = zeros(25,1);
theta = theta_dmp;


x = zeros(length([0:dt:T]),2);
time = [0:dt:T]';

%[time, x] = ode45(dynamics_mdl,[0:dt:T],[0;0]);
% for i = 1:number_rbf
%     phi(i,:) = rbf(x,theta(number_rbf+(i:i+1)),theta(number_rbf*(1+length(x_target))+(i:i+1)));
% end
% 
% [time, x] = ode45(dynamics_mdl,[0:dt:T],[0;0],OPTIONS,time,u')
%% Q-Learning Algorithmus
% s_present = S(3,1); %first initial state
figure
for k = 1:K %each epoch
    %% Exploration of policy
    % randomly initialize parametervector
    theta_temp(:,k) = theta(:,k) + sigma*randn(length(theta(:,k)),1); %max((1-sum(R_importance(max(idx,1),1))/max(idx+1e-2)),0.01)*
   % for i = 1:number_rbf % calculate rbf network
  %      phi(i,:) = rbf(x,theta_temp((number_rbf+2*(i-1))+(1:2),k),max(theta_temp((number_rbf*(1+length(x_target))+2*(i-1))+(1:2),k),zeros(2,1)));
  %  end
  %  u = theta_temp(1:number_rbf,k)'*phi;
  
    %[time, rollout] = ode45(dynamics_mdl,[0:dt:T],x_initial,OPTIONS,theta_temp(:,k),number_rbf);
    [rollout] = ode3(dynamics_mdl,[0:dt:T],x_initial,theta_temp(:,k),number_rbf,x_target);
    time = [0:dt:T];
    rollout = rollout(:,1:length(x_target));
   % rollout(:,1) =  mod(rollout(:,1),2*pi);%cos(rollout(:,1));% 
    rollout(:,3) = mod(rollout(:,3),2*pi);%mod(rollout(:,3),2*pi);
    
    if(mod(k,100)==0)
        subplot(3,1,1)
        plot(time,rollout)
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
    R(k) = sum(reward(rollout,x_target',alpha))/length(rollout); %rollout(end,:),x_target',alpha)+
    R_importance(k,:) = [R(k) k];
    R_importance = sortrows(R_importance,'descend');
    
    %% Update policy by PoWER
%     for i = 1:min(length(R_importance(:,1)),10)
%         delta_theta(:,i) = theta_temp(:,R_importance(i,2))-theta(:,k);
%     end
    idx = 1:min(length(R_importance(:,1)),10);
    delta_theta(:,idx) = ...
        theta_temp(:,R_importance(idx,2))-theta(:,k);
    
    theta(:,k+1) = theta(:,k) + sum(delta_theta.*R_importance(idx,1)',2)./sum(R_importance(idx,1)); 
    %theta_temp(:,R_importance(1,2));%

    
end
final_rollout = ode3(dynamics_mdl,[0:dt:T],x_initial,theta(:,k+1),number_rbf,x_target);
figure
plot(time,final_rollout)
disp('habe fertig')