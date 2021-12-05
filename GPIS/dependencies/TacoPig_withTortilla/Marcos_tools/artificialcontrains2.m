function V= artificialcontrains2(minx,maxx,offset)
% Validating offset ... 
s=size(offset,2);

if s == 1
    t=zeros(1,3);
    offset=t+offset;
 
end

if s==2 
    error('offset has to be a vector 1*3 ...')
end


V= zeros(3,8);
V(1,1:4)=minx(1); V(1,5:8)=maxx(1);
V(2,:) = repmat( [repmat(minx(2),1,2) repmat(maxx(2),1,2)],1,2 );
V(3,:) = repmat( [minx(3) maxx(3)],1,4);

% Adding offsets ...
V(1,1:4) =V(1,1:4) -offset(1);
V(1,5:8) =V(1,5:8) +offset(1);
V(2,[1,2,5,6]) = V(2,[1,2,5,6]) - offset(2);
V(2,[3,4,7,8]) = V(2,[3,4,7,8]) + offset(2);
V(3,[1,3,5,7])= V(3,[1,3,5,7]) - offset(3);
V(3,[2,4,6,8])= V(3,[2,4,6,8]) + offset(3);

% Displaying constrains ...
scatter3(V(1,:),V(2,:),V(3,:),'filled');

