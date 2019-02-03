set(groot,'defaultAxesFontSize',16)

filenames = dir('*.mat')
filenames =struct2cell(filenames);
filenames = filenames(1,:);
color = colormap('lines');


load('cartPole_10deg_7_H50.mat')
close all
h1 = figure
subplot(211)
time = 0:dt:T;
plot(time,[mu0([1,4])';yy(:,[1,4])])
hold all
plot([time(1) time(end)],[cost.target(1),cost.target(1)],'--k')%,'color',c(1,:))
plot([time(1) time(end)],[cost.target(4),cost.target(4)],'--k')%,'color',c(1,:))
grid
ylim([-1 4])
ylabel('states','interpreter','latex')
% hleg =legend('$x$ in [m]','$\dot{x}$ in [m/s]','location','northwest');
% set(hleg,'interpreter','latex')
subplot(212)
plot(time,[zeros(size(xx(1,end)));xx(:,end)],'color',color(3,:));
grid
ylabel('force [N]','interpreter','latex')
xlabel('time [s]','interpreter','latex')
set(h1,'Position',[1 1 500 350])
saveas(h1,'cp_quad','png')



disp(['Control Energy: ' num2str(1/length(xx(:,end))*sum(xx(:,end).^2))])
disp(['Optimality: ' num2str(sum(diag(yy*yy')))])

% figure
% subplot(221)
% errorbar(time, M{1}(1,:), ... % instead of time0:length(M{end}(1,:))-1
%       2*sqrt(squeeze(Sigma{1}(1,1,:))) );
%   hold all
% errorbar(time, M{end}(1,:), ...
%       2*sqrt(squeeze(Sigma{end}(1,1,:))) ); 
% grid
% subplot(222)
% errorbar(time, M{1}(2,:), ...
%       2*sqrt(squeeze(Sigma{1}(2,2,:))) );
%   hold all
% errorbar(time, M{end}(2,:), ...
%       2*sqrt(squeeze(Sigma{end}(2,2,:))) );  
% grid
% subplot(223)
% errorbar(time, M{1}(3,:), ...
%       2*sqrt(squeeze(Sigma{1}(3,3,:))) );
%   hold all
% errorbar(time, M{end}(3,:), ...
%       2*sqrt(squeeze(Sigma{end}(3,3,:))) ); 
% grid
% subplot(224)
% errorbar(time, M{1}(4,:), ...
%       2*sqrt(squeeze(Sigma{1}(4,4,:))) );
%   hold all
% errorbar(time, M{end}(4,:), ...
%       2*sqrt(squeeze(Sigma{end}(4,4,:))) );  
% xlim([time(1) time(end)]);
% grid
% 
% 
% 
% 
