
% Encapsulates all the possible mean functions we may implement
classdef GP_MeanFunc < handle
    
    methods(Abstract,Static)
        n = npar(D); % D is the number of dimensions
    end
    methods(Abstract)
        mu = eval(X, par);
    end
    methods

        % Some mean functions (projective ...)
        % need to know whether we are getting the mean of the 
        % observations or the mean of the function
        function mu = eval_y(this, X, par)
            mu = this.eval(X,par);
        end
        
        function g = gradient(this, X, par) 
            error([class(this),' does not implement gradients!']);
        end
    end
    
    
    
end    