set(groot,'defaultAxesFontSize',16)
filenames = dir('*.mat')
filenames =struct2cell(filenames);
filenames = filenames(1,:);
color = colormap('lines');
load(filenames{1})
time = 0:dt:T;
% figure
% subplot(211)
% 
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
load('sdm_1_H50.mat');
lat = cell(1,10);
for i=1:10
 [~,~,~,lat{i}] = rollout(gaussian(mu0, S0), policy, HH, plant, cost);
end

% if ~ishandle(4); figure(4); else set(0,'CurrentFigure',4); end; clf(4);
h1 = figure
plot([time(1) time(end)],[cost.target(1) cost.target(1)],'k--')
ldyno = length(dyno);
for i=1:1       % plot the rollouts on top of predicted error bars
% subplot(ceil(ldyno/sqrt(ldyno)),ceil(sqrt(ldyno)),i); hold on;
hold on
errorbar(time, M{j}(i,:), ...
  2*sqrt(squeeze(Sigma{j}(i,i,:))) );
for ii=1:10
  plot(time, lat{ii}(:,dyno(i)), 'color',color(3,:) );
end
plot(time, latent{j}(:,dyno(i)),'color','r');
axis tight
end
drawnow;
grid
ylabel('position [m]','interpreter','latex')
xlabel('time [s]','interpreter','latex')
set(h1,'Position',[1 1 430 220])
saveas(h1,'sdm','png')
set(h1,'Position',[1 1 500 350])
saveas(h1,'sdm_quad','png')




disp(['Control Energy: ' num2str(1/length(xx(:,end))*sum(xx(:,end).^2))])
disp(['Optimality: ' num2str(1/length(yy(:,end))*sum(diag(yy*yy')))])
% 
% figure
% subplot(211)
% errorbar( 0:length(M{1}(1,:))-1, M{1}(1,:), ...
%       2*sqrt(squeeze(Sigma{1}(1,1,:))) );
%   hold all
% errorbar( 0:length(M{end}(1,:))-1, M{end}(1,:), ...
%       2*sqrt(squeeze(Sigma{end}(1,1,:))) ); 
% grid
% subplot(212)
% errorbar( 0:length(M{1}(2,:))-1, M{1}(2,:), ...
%       2*sqrt(squeeze(Sigma{1}(2,2,:))) );
%   hold all
% errorbar( 0:length(M{end}(2,:))-1, M{end}(2,:), ...
%       2*sqrt(squeeze(Sigma{end}(2,2,:))) );  
% grid
% 
% 
