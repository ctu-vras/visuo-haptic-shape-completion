


% Sum of gaussian basis functions
% weights = [Border1 Border2 ... Lengthscale1 Lengthscale2 Lengthscale3...]

function y = stepchange1D(x,NumBorders,params)

[D N] = size(x);
% Return the required number of weights if none are supplied
if nargin <3  
   if numel(NumBorders) ~=1
       error('NumBasFun must be a natural number')
   end
  
   y =  NumBorders*2+1;
   return  
end


Borders = params(1:NumBorders);
Lengths = params(NumBorders+1:end);

[Borders ID]=sort(Borders);
Lengths = Lengths(ID);
y = zeros(1,N);

y(x<=Borders(1)) = Lengths(1);


for i=1:NumBorders
    
    y(x>Borders(i))=Lengths(i+1);
end

% y(x>=Borders(end)) = Lengths(end);


