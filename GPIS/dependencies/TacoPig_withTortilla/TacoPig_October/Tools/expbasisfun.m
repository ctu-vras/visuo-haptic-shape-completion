


% Sum of gaussian basis functions
% weights = [w0 w1 w2 ... meanexp1dim1 meanexp1dim2.... 
% meanexp2dim1 meanexp2dim2.... 
% lengthexp1dim1 lengthexp1dim2 ... lengthexp2dim1...]

function y = expbasisfun(x,NumBasFun,params)

[D N] = size(x);
% Return the required number of weights if none are supplied
if nargin <3  
   if numel(NumBasFun) ~=1
       error('NumBasFun must be a natural number')
   end
   D = size(x,1) ;
   y =  NumBasFun*(D*2+1)+1;
   return  
end

W = [params(1)  params(2:NumBasFun+1)]; %Weights of the gaussians
Means =  params(NumBasFun+2:NumBasFun+1+(NumBasFun*D)); %Means of the gaussians
Means = reshape(Means,D,NumBasFun);
Lengthscales =  params(NumBasFun+2+(NumBasFun*D):end);
Lengthscales = reshape(Lengthscales,D,NumBasFun);

Phi = [ones(1,size(x,2))];

for i = 1:NumBasFun
    Meani = Means(:,i);
    Dist = x - Meani(:,ones(N,1));
    Sig = diag(Lengthscales(:,i)).^2;
    Phi(i+1,:)=diag(exp(-0.5*Dist'*inv(Sig)*Dist))';
end

y = W*Phi;
