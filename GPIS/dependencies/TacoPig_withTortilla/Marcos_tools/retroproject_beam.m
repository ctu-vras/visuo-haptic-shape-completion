
function [pointout pointin] =retroproject_beam (X,nav,index)
deltas = X(index,:) - nav(index,:); % with respect to X
ma = sqrt( deltas(:,1).^2 + deltas(:,2).^2 + deltas(:,3).^2);
magnitude=repmat(ma,1,3);
theta=acosd(deltas./magnitude);
%result= nav(index,:) + (cosd(theta) .* (magnitude+0.5)); 
% or result= X(index,:) - (cosd(theta) * magnitude); 
pointout= nav(index,:) + (cosd(theta) .* (magnitude-0.5)); % points out
pointin=  nav(index,:) + (cosd(theta) .* (magnitude+0.2)); % and this points in/out %from 0.01 to 0.1

%plotting
scatter3([pointout(:,1)], [pointout(:,2)], [pointout(:,3)],100,'filled','r')
scatter3([pointin(:,1)], [pointin(:,2)], [pointin(:,3)],100,'filled','y')