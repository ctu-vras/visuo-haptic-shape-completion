classdef GP_SqExpCov_Barrier < GP_CovFunc
   
    % Most covariance functions will be static
    methods(Static) 
        
        function n_theta = npar(D)
            n_theta = D+2; % each dimension + signal variance
        end
        
        function K = eval(X1, X2, par)
            Barrier = par(end-1);
            [D,N1] = size(X1); %number of points in X1
            N2 = size(X2,2); %number of points in X2
            if D~=size(X2,1)
                error('Dimensionality of X1 and X2 must be the same');
            end
            if (length(par)~=D+2)
                error('Wrong number of hyperparameters for BarrierSqExp');
            end
            
            BarrierLess = double((X1<Barrier))'*double((X2<Barrier));
            BarrierGreater = double((X1>=Barrier))'*double((X2>=Barrier));
            BarrierID = BarrierLess+BarrierGreater;
            %Compute weighted squared distances:
            w = par(1:D)'.^(-2);
            XX1 = sum(w(:,ones(1,N1)).*X1.*X1,1);
            XX2 = sum(w(:,ones(1,N2)).*X2.*X2,1);
            X1X2 = (w(:,ones(1,N1)).*X1)'*X2;
            XX1T = XX1';
            % numerical effects can drive z slightly negative 
            z = max(0,XX1T(:,ones(1,N2)) + XX2(ones(1,N1),:) - 2*X1X2);
            K = par(end)^2 * exp(-0.5*z);
            K = K.*BarrierID;
        end
        
%         function [g] = gradient(X, par)
%             
%             % Same as K?
%             Kg = GP_SqExpCov.eval(X, X, par);
%             
%             [d,n] = size(X);
%             g = cell(1,d+1);
%             for i=1:d
%                 %Compute weighted squared distance
%                 row = X(i,:);
%                 XX = row.*row;
%                 XTX = row'*row;
%                 XX = XX(ones(1,n),:);
%                 z = max(0,XX+XX'-2*XTX);
%                 w = par(i)^(-3);
%                 g{i} = w*Kg.*z;
%             end
%             g{d+1} = Kg*(2/par(end));
%         end
        
        
        % Also overload the point covariance kx*x* - its trivial
%         function v = pointval(x_star, par)
%              Barrier = par(end-1);
%               BarrierLess = double((x_star<Barrier))'*double((x_star<Barrier));
%             BarrierGreater = double((x_star>=Barrier))'*double((x_star>=Barrier));
%             BarrierID = BarrierLess+BarrierGreater;
%             v = par(end).^2 * ones(1,size(x_star,2));
%             v = v.*diag(BarrierID);
%         end
        
    end
end
