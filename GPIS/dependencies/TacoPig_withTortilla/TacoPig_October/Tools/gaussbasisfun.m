

% Single multi-variate gaussian basis function
% x = D x N  where D is dimension number and N is number of points
% params = [mean1dim1 mean1dim2 ...length1dim1 length1dim2... mean2dim1 mean2dim2 ...length2dim1 length2dim2...]
function y = gaussbasisfun(x,params)

[D N] = size(x);
% Return the required number of weights if none are supplied
if nargin <2  
   D = size(x,1) ;
   y =  D*2; %Mean and lengthscale for each dimension
   return  
end

Mean =  params(1:end/2)'; %Means of the gaussians
Lengthscales =  params(end/2+1:end);

Dist = x - Mean(:,ones(N,1));
Sig = diag(Lengthscales).^2;
y=diag(exp(-0.5*Dist'*inv(Sig)*Dist))';

