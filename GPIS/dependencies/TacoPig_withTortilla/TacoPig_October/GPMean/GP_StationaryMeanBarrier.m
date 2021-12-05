% Constant Mean GP Function
% This is an example implementation of a polynomial mean function...

classdef GP_StationaryMeanBarrier < GP_MeanFunc
    methods(Static)
        function n = npar(~) 
            n = 3; % intercept and gradients in each dimension
        end
        
        function mu = eval(X, par) 
            N1 = sum((X<par(3)));
            N2 = sum((X>=par(3)));
            mu(X<par(3)) = par(1)*ones(1,N1);
            mu(X>=par(3)) = par(2)*ones(1,N2);
        end
        
%         function g = gradient(X, ~) 
%             N = size(X,2);
%             g = cell(1,1);
%             g{1} = ones(1,N);
%         end
    end
end    


