
function boundaries = artificialcontrains (minx, maxx, minth,maxth)
%th:  positive threshold for points outside, negative for points inside
%minx: minimum boundaries for 3D points clouds in x,y,z
%maxx: maximum boundaries for 3D points clouds in x,y,z

minx= minx-minth;  
maxx = maxx+maxth;


boundaries= [repmat(minx(1),1,4) repmat(maxx(1),1,4); ...
    repmat(minx(2),1,2) repmat(maxx(2),1,2) repmat(minx(2),1,2) ... 
    repmat(maxx(2),1,2); repmat([minx(3) maxx(3)],1,4)];

scatter3(boundaries(1,:)', boundaries(2,:)', boundaries(3,:)', 'filled');
