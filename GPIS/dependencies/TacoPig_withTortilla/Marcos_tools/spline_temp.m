R = 5
spline = @(r) (1/12)*( 2*abs(r).^3 - 3*R*r.^2+R^3)
x=linspace(0,10,100)

centre=round(length(x)/2);
r=x(centre)-x;

y = spline(r)
figure;plot(r,y)

y(x>R)=0
figure;plot(r,y)