%% draw_rollout_cp.m
% *Summary:* Script to draw the most recent trajectory of the cart-pole
% system together with the predicted uncertainties around the tip of the 
% pendulum
%
% Copyright (C) 2008-2013 by 
% Marc Deisenroth, Andrew McHutchon, Joe Hall, and Carl Edward Rasmussen.
%
% Last modified: 2013-05-20
%
%% High-Level Steps
% # For each time step, plot the observed trajectory and the predicted
% means and covariances of the Cartesian coordinates of the tip of the
% pendulum

%% Code

% Loop over states in trajectory (= time steps)
for r = 1:size(xx,1)
  if exist('j','var') && ~isempty(M{j})
    draw_sdm(latent{j}(r,1), latent{j}(r,2), cost,  ...
      ['trial # ' num2str(j+J) ', T=' num2str(H*dt) ' sec'], ...
      ['total experience (after this trial): ' num2str(dt*size(x,1)) ...
      ' sec'], M{j}(:,r), Sigma{j}(:,:,r));
  else
      draw_sdm(latent{jj}(r,1), latent{jj}(r,2), cost,  ...
      ['(random) trial # ' num2str(jj) ', T=' num2str(H*dt) ' sec'], ...
      ['total experience (after this trial): ' num2str(dt*size(x,1)) ...
      ' sec'])
  end
  pause(dt);
end
