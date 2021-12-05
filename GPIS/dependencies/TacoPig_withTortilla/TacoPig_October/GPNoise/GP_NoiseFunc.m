% Covariance function
% Eval:
% X1 - D x N input points 1
% X2 - D x M input points 2
% par: 1 x N vector of hyperparameters
classdef GP_NoiseFunc < handle
    methods (Abstract)
        n_theta = npar(this); 
        noise = eval(this, X1, X2, theta);
    end
    
    methods
        % Our design splits the covariance function into its usage cases
        
        % Special case for getting Kx*x*
        % Overload this if there is a more efficient way for your function
%         function v = pointval(this, x_star, theta)
%             nstar = size(x_star,2);
%             v = zeros(1,nstar);
%             for i=1:nstar
%                 xx = x_star(:,i);
%                 v(i) = this.eval(xx,xx, theta);
%             end
%         end
        
        function g = gradient(this, X, par)
            error([class(this),' does not implement gradients!']);
        end
        
    end
end    
