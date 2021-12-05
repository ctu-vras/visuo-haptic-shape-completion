% Define the interface for any GP model we may implement
% X - D x N input points
% y - 1 x N training outputs
% x_star - D x M input points
% par: 1 x N vector of hyperparameters

% If you want to delve into classification (EP etc)
% we don't put it all in the one file, but use inheritance
% eg for multitaks....

classdef GP_Model < handle
    % Public member variables:
    % Initially at least, keep everything public so when people need to 
    % play with their GP variables they dont have to stop using this code
    properties
        MeanFn
        CovFn
        NoiseFn
        X
        y
    end
    
    methods (Abstract)
        theta = learn(obj);
        solve(obj);
        [mu, var] = query(obj, xstar);
        [objective, objectiveg] = objectfun(theta);
        optimset(varargin);
    end
    
    
end    