


% Sum of gaussian basis functions
% weights = [Border1 Border2 ... Lengthscale1 Lengthscale2 Lengthscale3...]

function y = stepchange1DTransitions(x,NumTrans,params)

[D N] = size(x);
% Return the required number of weights if none are supplied
if nargin <3  
   if numel(NumTrans) ~=1
       error('NumBasFun must be a natural number')
   end
  
   y =  NumTrans*3+1;
   return  
end


Mean = params(1:NumTrans);
Width = params(NumTrans+1:2*NumTrans);
LengthsInTrans = params(2*NumTrans+1:end-1);
Bias = params(end);


y = Bias*ones(1,N);

for i=1:NumTrans
    [x>(Mean(i)-(Width(i)/2))].*[(x<Mean(i)+(Width(i)/2))]
    y(find([x>(Mean(i)-(Width(i)/2))].*[(x<Mean(i)+(Width(i)/2))]==1))=LengthsInTrans(i);
end
% y(x>=Borders(end)) = Lengths(end);


