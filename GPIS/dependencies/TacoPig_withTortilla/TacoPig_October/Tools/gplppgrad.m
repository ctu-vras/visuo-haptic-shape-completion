function [nplp,ng] = gplppgrad(params,X,y,gp)

if isfield(gp,'mfun')
    mpar  = params(gp.mappar.mpar);
    mfun  = gp.mfun;
    mgfun = gp.mgfun;
else
    mfun  = [];
    mgfun = [];
end
noise = params(end);
kpar  = params(1:gp.numberOfCovarianceParameters);
kfun  = gp.covarianceFunction;

N = size(X,2);

% check if mean is zero
if isempty(mfun)
    y = y - mean(y);
else
    m    = feval(mfun,X,mpar);
end

K = feval(kfun,X,X,kpar);

%Cholesky factorisation for covariance inversion
if issparse(K)
    I      = speye(N);
    A      = K+(noise^2)*I;
    L      = CholeskyJitter(A)';
    alpha  = L'\(L\(y)');
else
    I     = eye(N);
    L     = CholeskyJitter(K+(noise^2)*I); 
    alpha = SolveCholesky(L,(y)');
end    
    
% precompute for convenience
if issparse(L)
    invK  = L'\(L\I);
else
    invK  = SolveCholesky(L,I);
end

%Compute the negative predictive log probability
R = invK*y';
nplp = 0;
for i = 1:N
    %Leave out training case i
    mu_i = y(i)-R(i)/invK(i,i);
    sigma_i = sqrt(1/invK(i,i));
    nplp_i = log(sigma_i^2)/2 +(y(i)-mu_i)^2/(2*sigma_i^2)+log(2*pi)/2;
    nplp =nplp + nplp_i;
end
if nargout == 2
    %Compute the gradient for covariance hyperparameters

    kgrad = cell(1,length(kpar));
    ng    = zeros(1,length(params));

    %Compute the gradient numerically if the options exists
    %and if it is set to numerical.
    if gp.optimizerNumericalGradient
        delta = 0.01;
        for j = 1:length(kpar)
            %Modify hyperparameters by delta.
            paramsDelta = params;
            paramsDelta(j) = params(j)+delta;
            nplpDelta = gplppgrad(paramsDelta,X,y,gp);
            ng(j) = (nplpDelta-nplp)/delta;
        end
        
        delta = 0.001;
        paramsDelta = params;
        paramsDelta(end) = params(end)+delta;
        nplpDelta = gplppgrad(paramsDelta,X,y,gp);
        ng(end) = (nplpDelta-nplp)/delta;
        
        return;
    end

    %Calculate the gradient using derivate expression.
    Z = cell(1,length(kpar));
    for j = 1:length(kpar)
        kgrad{j} = feval(kfun,X,X,kpar,j);
        Z{j} = invK*kgrad{j};
    end


    for j = 1:length(kgrad)
        for i = 1:N
            h = Z{j}*alpha;
            U = Z{j}*invK;
            ng_i = ((alpha(i)*h(i))-(1+(alpha(i)^2)/invK(i,i))*(U(i,i)/2))/invK(i,i);
            ng(j) = ng(j)+ng_i;
        end
    end

    for i = 1:N
        Z_noise = invK;
        h_noise = Z_noise*alpha;
        U_noise = Z_noise*invK;
        ng_noise_i = ((alpha(i)*h_noise(i))-(1+(alpha(i)^2)/invK(i,i))*(U_noise(i,i)/2))/invK(i,i);
        ng(end) = ng(end) + ng_noise_i;
    end
end