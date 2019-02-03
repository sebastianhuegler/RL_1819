filenames = dir('*.mat')
filenames =struct2cell(filenames);
filenames = filenames(1,:);
color = colormap('lines');
load(filenames{1})

figure
subplot(211)
time = 0:dt:T;
plot(time,[zeros(size(yy(1,:)));yy])
hold all
plot([0 time(end)],[cost.target(1),cost.target(1)],'--k')%,'color',c(1,:))
plot([0 time(end)],[cost.target(2),cost.target(2)],'--k')%,'color',c(1,:))
grid
ylabel('system states','interpreter','latex')
hleg=legend('$x$ in [m]','$\dot{x}$ in [m/s]');
set(hleg,'interpreter','latex')
subplot(212)
plot(time,[zeros(size(xx(1,end)));xx(:,end)],'color',color(3,:));
grid
ylabel('force in [N]','interpreter','latex')
xlabel('time [s]','interpreter','latex')

disp(['Control Energy: ' num2str(1/length(xx(:,end))*sum(xx(:,end).^2))])
disp(['Optimality: ' num2str(sum(diag(yy*yy')))])
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
