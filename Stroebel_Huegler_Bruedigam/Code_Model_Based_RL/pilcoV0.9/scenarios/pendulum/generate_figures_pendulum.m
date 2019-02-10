filenames = dir('*.mat')
filenames =struct2cell(filenames);
filenames = filenames(1,:);
color = colormap('lines');
load(filenames{1})
close all

% figure
% subplot(211)
% time = 0:dt:T;
% plot(time,[zeros(size(yy(1,:)));yy])
% hold all
% plot([0 time(end)],[cost.target(1),cost.target(1)],'--k')%,'color',c(1,:))
% plot([0 time(end)],[cost.target(2),cost.target(2)],'--k')%,'color',c(1,:))
% grid
% ylabel('system states','interpreter','latex')
% hleg=legend('$x$ in [m]','$\dot{x}$ in [m/s]');
% set(hleg,'interpreter','latex')
% subplot(212)
% plot(time,[zeros(size(xx(1,end)));xx(:,end)],'color',color(3,:));
% grid
% ylabel('force in [N]','interpreter','latex')
% xlabel('time [s]','interpreter','latex')

disp(['Control Energy: ' num2str(1/length(xx(:,end))*sum(xx(:,end).^2))])
disp(['Optimality: ' num2str(sum(diag(yy*yy')))])

% figure
% subplot(211)
% errorbar(time, M{1}(1,:), ... % instead of time0:length(M{end}(1,:))-1
%       2*sqrt(squeeze(Sigma{1}(1,1,:))) );
%   hold all
%   errorbar(time, M{3}(1,:), ...
%       2*sqrt(squeeze(Sigma{3}(1,1,:))) ); 
% errorbar(time, M{end}(1,:), ...
%       2*sqrt(squeeze(Sigma{end}(1,1,:))) ); 
% grid
% subplot(212)
% errorbar(time, M{1}(2,:), ...
%       2*sqrt(squeeze(Sigma{1}(2,2,:))) );
%   hold all
% errorbar(time, M{end}(2,:), ...
%       2*sqrt(squeeze(Sigma{end}(2,2,:))) );  
% grid




load('pendulum_1_H50.mat');
lat = cell(1,10);
for i=1:10
 [~,~,~,lat{i}] = rollout(gaussian(mu0, S0), policy, HH, plant, cost);
end
% if ~ishandle(4); figure(4); else set(0,'CurrentFigure',4); end; clf(4);
h1 = figure
subplot(211)
plot([time(1) time(end)],-[cost.target(2) cost.target(2)],'k--')
hold all
ldyno = length(dyno);
for i=2:2       % plot the rollouts on top of predicted error bars
% subplot(ceil(ldyno/sqrt(ldyno)),ceil(sqrt(ldyno)),i); hold on;
errorbar( time, M{j}(i,:), ...
  2*sqrt(squeeze(Sigma{j}(i,i,:))) );
for ii=1:10
  plot( time, lat{ii}(:,dyno(i)),'color',color(3,:));
end
plot( time, latent{j}(:,dyno(i)),'r');
axis tight
end
drawnow;

load('pendulum_3_H50.mat');
lat = cell(1,10);
for i=1:10
 [~,~,~,lat{i}] = rollout(gaussian(mu0, S0), policy, HH, plant, cost);
end
ylabel('angle [rad]','interpreter','latex')
grid
subplot(212)%figure
plot([time(1) time(end)],-[cost.target(2) cost.target(2)],'k--')
hold all
ldyno = length(dyno);
for i=2:2       % plot the rollouts on top of predicted error bars
% subplot(ceil(ldyno/sqrt(ldyno)),ceil(sqrt(ldyno)),i); hold on;
errorbar( time, M{j}(i,:), ...
  2*sqrt(squeeze(Sigma{j}(i,i,:))) );
for ii=1:10
  plot( time, lat{ii}(:,dyno(i)),'color',color(3,:));
end
plot( time, latent{j}(:,dyno(i)),'r');
axis tight
end
drawnow;
grid
ylabel('angle [rad]','interpreter','latex')
xlabel('time [s]','interpreter','latex')
set(h1,'Position',[1 1 430 220])
saveas(h1,'pend','png')
set(h1,'Position',[1 1 500 350])
saveas(h1,'pend_quad','png')

