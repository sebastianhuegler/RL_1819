set(groot,'defaultAxesFontSize',16)
%% Mass-Spring-Damper
load('massspringdamper_superbei147_40rbf_10Nforce.mat')
for ku= 1:100:length(theta(1,:))
% Simulate DMP
[y] = ode3(@dmp,time,dmp_settings.init_state,theta(:,ku),dmp_settings);  %time: simulationszeit
if(control_mode == 0)
%without controller
policy = dmp_policy(y,theta(:,ku),dmp_settings);%y(:,1:2);%
policy_plot(:,:,ceil(ku/100)) = policy;
elseif(control_mode == 1)
%with controller
policy(:,:,ceil(ku/100)) = [y(:,1:end-1) dmp_policy(y,theta(:,ku),dmp_settings);];%[x_target(1)*ones(length(y(:,1)),1),zeros(length(y(:,1)),1),zeros(length(y(:,1)),1)];%
end
final_rollout = ode3(dynamics_mdl,time,x_initial,policy,time,mdl,control_mode);
final_rollout_plot(:,:,ceil(ku/100)) = final_rollout;
R_final(ceil(ku/100))= sum(reward(final_rollout(:,reward_states),x_target(reward_states)',alpha))/length(final_rollout);
end

x_target = 1.5;
h1 = figure
subplot(211)
plot(time,final_rollout_plot(:,1,3),'LineWidth', 2) %trial 300
hold all;
plot(time,final_rollout_plot(:,1,10),'LineWidth', 2) %trial 1000
plot(time,final_rollout_plot(:,1,20),'LineWidth', 2) %trial 2000
plot(time,final_rollout_plot(:,1,30),'LineWidth', 2) %trial 3000
plot(time,final_rollout_plot(:,1,50),'LineWidth', 2) %trial 5000 
plot([time(1) time(end)],[x_target x_target],'k--','LineWidth', 1)
%plot(time,final_rollout(:,1,100),'LineWidth', 2) %trial 10.000
grid on
xlabel('time [sec]','interpreter','latex')
ylabel('system output [m]','interpreter','latex') %MASS-SPRING-DAMPER
legend('#300','#1000','#2000','#3000','#5000','location','southeast')%,'orientation','horizontal')
%legend('boxoff')
ylim([-0.1  1.6])
%set(gca,'FontSize',16)

subplot(212)
plot(R_importance(R_importance(:,2)<=5000,2),R_importance(R_importance(:,2)<=5000,1),'.')
xlabel('trials','interpreter','latex')
ylabel('reward','interpreter','latex')
grid on

% %POLICY:
% plot(time,policy_plot(:,1,3),'LineWidth', 2) %trial 300
% hold all;
% plot(time,policy_plot(:,1,10),'LineWidth', 2) %trial 1000
% plot(time,policy_plot(:,1,30),'LineWidth', 2) %trial 3000
% plot(time,policy_plot(:,1,50),'LineWidth', 2) %trial 5000 
% %plot(time,policy_plot(:,1,100),'LineWidth', 2) %trial 10.000
% grid on
% xlabel('time [sec]','interpreter','latex')
% ylabel('policy [N]','interpreter','latex')
% %legend('300 trials','1000 trials','3000 trials','5000 trials','10.000 trials')
% %set(gca,'FontSize',16)

set(h1,'Position',[1 1 500 450])
saveas(h1,'julia_sdm_quad','png') %MASS-SPRING-DAMPER

%% PENDULUM
load('pendulum_super_40rbf_10Nforce.mat')
for ku= 1:100:length(theta(1,:))
% Simulate DMP
[y] = ode3(@dmp,time,dmp_settings.init_state,theta(:,ku),dmp_settings);  %time: simulationszeit
if(control_mode == 0)
%without controller
policy = dmp_policy(y,theta(:,ku),dmp_settings);%y(:,1:2);%
policy_plot(:,:,ceil(ku/100)) = policy;
elseif(control_mode == 1)
%with controller
policy(:,:,ceil(ku/100)) = [y(:,1:end-1) dmp_policy(y,theta(:,ku),dmp_settings);];%[x_target(1)*ones(length(y(:,1)),1),zeros(length(y(:,1)),1),zeros(length(y(:,1)),1)];%
end
final_rollout = ode3(dynamics_mdl,time,x_initial,policy,time,mdl,control_mode);
final_rollout_plot(:,:,ceil(ku/100)) = final_rollout;
R_final(ceil(ku/100))= sum(reward(final_rollout(:,reward_states),x_target(reward_states)',alpha))/length(final_rollout);
end

x_target = -pi;
h2=figure
subplot(211)
plot(time,final_rollout_plot(:,1,3),'LineWidth', 2) %trial 300
hold all;
plot(time,final_rollout_plot(:,1,10),'LineWidth', 2) %trial 1000
plot(time,final_rollout_plot(:,1,20),'LineWidth', 2) %trial 2000
plot(time,final_rollout_plot(:,1,30),'LineWidth', 2) %trial 3000
plot(time,final_rollout_plot(:,1,50),'LineWidth', 2) %trial 5000 
%plot(time,final_rollout(:,1,100),'LineWidth', 2) %trial 10.000
plot([time(1) time(end)],[x_target x_target],'k--','LineWidth', 1)

grid on
xlabel('time [sec]','interpreter','latex')
ylabel('system output [rad]','interpreter','latex')%PENDULUM
legend('#300','#1000','#2000','#3000','#5000','location','southeast')%,'orientation','horizontal')
%legend('boxoff')

%set(gca,'FontSize',16)

subplot(212)
plot(R_importance(R_importance(:,2)<=5000,2),R_importance(R_importance(:,2)<=5000,1),'.')
xlabel('trials','interpreter','latex')
ylabel('reward','interpreter','latex')
grid on

% %POLICY
% plot(time,policy_plot(:,1,3),'LineWidth', 2) %trial 300
% hold all;
% plot(time,policy_plot(:,1,10),'LineWidth', 2) %trial 1000
% plot(time,policy_plot(:,1,30),'LineWidth', 2) %trial 3000
% plot(time,policy_plot(:,1,50),'LineWidth', 2) %trial 5000 
% %plot(time,policy_plot(:,1,100),'LineWidth', 2) %trial 10.000
% grid on
% xlabel('time [sec]','interpreter','latex')
% ylabel('policy [N]','interpreter','latex')
% %legend('300 trials','1000 trials','3000 trials','5000 trials','10.000 trials')
% %set(gca,'FontSize',16)

set(h2,'Position',[1 1 500 450])
saveas(h2,'julia_pend_quad_with_2000trails','png') %PENDULUM

%% Cart-Pole - plotte init: 10°, 90°, 180°
h3 = figure
 
subplot(311)
for ku=1:3
    if ku ==1
        load('cartpole_superbleibtda_40rbf_10deginit_10Nforce.mat')
    elseif ku==2
        load('cartpole_superlaeuftweg_40rbf_90deginit_10Nforce.mat')
    else
        load('cartpole_superlaeuftweg_40rbf_180deginit_10Nforce')
    end
plot(time,final_rollout(:,[1]),'LineWidth',2)
hold all
end
grid on
%xlabel('time [sec]','interpreter','latex')
ylabel('pos. [m]','interpreter','latex')
legend('\theta_0 = 10°','\theta_0 = 90°','\theta_0 = 180°','location','southeast')

subplot(312)
for ku=1:3
    if ku ==1
        load('cartpole_superbleibtda_40rbf_10deginit_10Nforce.mat')
    elseif ku==2
        load('cartpole_superlaeuftweg_40rbf_90deginit_10Nforce.mat')
    else
        load('cartpole_superlaeuftweg_40rbf_180deginit_10Nforce')
    end
plot(time,final_rollout(:,[3]),'LineWidth',2)
hold all
end
grid on
%xlabel('time [sec]','interpreter','latex')
ylabel('angle [rad]','interpreter','latex')
% legend('10°: pole','90°: pole','180°: pole','location','southeast')

subplot(313)
for ku=1:3
     if ku ==1
        load('cartpole_superbleibtda_40rbf_10deginit_10Nforce.mat')
    elseif ku==2
        load('cartpole_superlaeuftweg_40rbf_90deginit_10Nforce.mat')
    else
        load('cartpole_superlaeuftweg_40rbf_180deginit_10Nforce')
    end
plot(time,policy,'LineWidth',2)
hold all
end
grid on
xlabel('time [sec]','interpreter','latex')
ylabel('policy [N]','interpreter','latex')
% legend('10°','90°','180°','location','southeast')


set(h3,'Position',[1 1 500 450])
saveas(h3,'cp_quad','png')

%% Cart-Pole - plotte init: 10°/90°/180°
%load('cartpole_superbleibtda_40rbf_10deginit_10Nforce.mat') % 10° init
%load('cartpole_superlaeuftweg_40rbf_90deginit_10Nforce') % 90° init
load('cartpole_superlaeuftweg_40rbf_180deginit_10Nforce') % 180° init

for ku= 1:100:length(theta(1,:))
% Simulate DMP
[y] = ode3(@dmp,time,dmp_settings.init_state,theta(:,ku),dmp_settings);  %time: simulationszeit
if(control_mode == 0)
%without controller
policy = dmp_policy(y,theta(:,ku),dmp_settings);%y(:,1:2);%
policy_plot(:,:,ceil(ku/100)) = policy;
elseif(control_mode == 1)
%with controller
policy(:,:,ceil(ku/100)) = [y(:,1:end-1) dmp_policy(y,theta(:,ku),dmp_settings);];%[x_target(1)*ones(length(y(:,1)),1),zeros(length(y(:,1)),1),zeros(length(y(:,1)),1)];%
end
final_rollout = ode3(dynamics_mdl,time,x_initial,policy,time,mdl,control_mode);
final_rollout_plot(:,:,ceil(ku/100)) = final_rollout;
R_final(ceil(ku/100))= sum(reward(final_rollout(:,reward_states),x_target(reward_states)',alpha))/length(final_rollout);
end

x_target1 = 0;
x_target2 = pi;

h4 = figure
subplot(311) %Verlauf: cart
plot(time,final_rollout_plot(:,[1],3),'LineWidth', 2) %final_rollout_plot
hold all; grid on
plot(time,final_rollout_plot(:,[1],10),'LineWidth', 2)
plot(time,final_rollout_plot(:,[1],20),'LineWidth', 2)
plot(time,final_rollout_plot(:,[1],30),'LineWidth', 2)
plot(time,final_rollout_plot(:,[1],50),'LineWidth', 2)
%plot(time,final_rollout_plot(:,[1],100),'LineWidth', 2)
plot([time(1) time(end)],[x_target1 x_target1],'k--','LineWidth', 1)
ylabel('pos. [m]','interpreter','latex')
%legend('#300','#1000','#2000','#3000','#5000','location','southeast')%,'orientation','horizontal')


subplot(312) %Verlauf: pole
plot(time,final_rollout_plot(:,[3],3),'LineWidth', 2)
hold all; grid on;
plot(time,final_rollout_plot(:,[3],10),'LineWidth', 2)
plot(time,final_rollout_plot(:,[3],20),'LineWidth', 2)
plot(time,final_rollout_plot(:,[3],30),'LineWidth', 2)
plot(time,final_rollout_plot(:,[3],50),'LineWidth', 2)
%plot(time,final_rollout_plot(:,[3],100),'LineWidth', 2)
plot([time(1) time(end)],[x_target2 x_target2],'k--','LineWidth', 1)
ylabel('angle [rad]','interpreter','latex')

subplot(313)
plot(R_importance(R_importance(:,2)<=5000,2),R_importance(R_importance(:,2)<=5000,1),'.')
xlabel('trials','interpreter','latex')
ylabel('reward','interpreter','latex')
grid on

% % POLICY
% subplot(313) %Verlauf: policy
% plot(time,policy_plot(:,1,3),'LineWidth', 2)
% hold all; grid on
% plot(time,policy_plot(:,1,10),'LineWidth', 2)
% plot(time,policy_plot(:,1,30),'LineWidth', 2)
% plot(time,policy_plot(:,1,50),'LineWidth', 2)
% %plot(time,policy_plot(:,1,100),'LineWidth', 2)
% xlabel('time [sec]','interpreter','latex')
% ylabel('policy [N]','interpreter','latex')

set(h4,'Position',[1 1 500 450])
saveas(h4,'julia_cp10_quad_180init','png')