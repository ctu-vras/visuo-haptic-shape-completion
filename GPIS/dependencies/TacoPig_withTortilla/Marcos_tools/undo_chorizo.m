function [x y z]= undo_chorizo(pos, res,scale)
% pos= nx1 Vector with values to be transformed to 3D 
% res= resolution of the 3D matrix [x y z]
% x: all positions in x, nx1
% y: positions in y, nx1
% z: positions in z, nx1

s= size(pos,1);
x= zeros(s,1);
y=x;
z=x;

for i=1:s
    x(i)=mod(pos(i),res); 
    y(i)=floor(mod(pos(i),res^2)/res);
    z(i)=floor(pos(i)/res^2);
end

f= @(axe,res,i) axe.min + ((axe.max - axe.min)* (i/res));
x= f(scale.x,res,x);
y= f(scale.y,res,y);
z= f(scale.z,res,z);