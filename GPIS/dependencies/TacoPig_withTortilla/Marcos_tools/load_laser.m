function [X y minx maxx rp]= load_laser(filename)

%compresorpath='/home/marcos/Desktop/gp_matlab_original/datasets/laser_clean/thin_data/tcalib_compresor_thin056_withnav.csv';
temp=load(filename);
X=temp(:,5:7);
offset = [0.108987 0.008302 -0.919726]; %% put gps on the correct position
nav= temp(:,11:13);
nav=[(nav(:,1)+offset(1)) (nav(:,2)+offset(2)) (nav(:,3)+offset(3))];


% make that to local coordinates
%tt=[max(nav(:,1)) max(nav(:,2)) max(X(:,3))]; % max in Z;
index= X(:,3)==min(X(:,3));
tt= X(index,:); % reference point (rp)
rp=tt;
X=[(X(:,1)-tt(1)) (X(:,2)-tt(2)) (X(:,3)-tt(3))];
nav=[(nav(:,1)-tt(1)) (nav(:,2)-tt(2)) (nav(:,3)-tt(3))];

%plotting this points in 3D
%figure(2)
%scatter3(X(:,1),X(:,2),X(:,3),'filled');
%set(gca,'ZDir','reverse')
%hold on
%scatter3(nav(:,1),nav(:,2),nav(:,3),'filled','r');

%set(gca,'ZDir','reverse') % upsidedown
%scatter3(X(index,1),X(index,2),X(index,3),100,'filled','m');
%set(gca,'ZDir','reverse')
%hold on
%scatter3(nav(index,1),nav(index,2),nav(index,3),100,'filled','m')
% title('Original Raw Data')
% getting boundaries
if (size(X,1) >1)
    minx= min(X);   
    maxx = max(X);
elseif (size(X,1) ==1)
        minx=X;
        maxx=X;
end

        


artificialcontrains2(minx,maxx,0);
hold on
points_out(:,1:8) = artificialcontrains2(minx,maxx,2);
points_out(:,9:16) = artificialcontrains2(minx,maxx,... 
    [0.5 0.5 -(maxx(3)-minx(3))/2]);
points_out(:,17:24) = artificialcontrains2(minx,maxx,... 
    [0.8 -(maxx(2)-minx(2))/4 -(maxx(3)-minx(3))/4]);
points_out(:,25:32) = artificialcontrains2(minx,maxx, ... 
    [-(maxx(1)-minx(1))/6 -(maxx(2)-minx(2))/6 (maxx(2)-minx(2))/6]);
points_out(:,33:40) = artificialcontrains2(minx,maxx,...
    [-(maxx(1)-minx(1))/5 -(maxx(2)-minx(2))/5 1]);

hold on
points_in(:,1:8)= artificialcontrains2(minx,maxx, ...
    [-(maxx(1)-minx(1))/2 -(maxx(2)-minx(2))/2 -(maxx(3)-minx(3))/2]);
  t=0;  

line([X(index,1) nav(index,1)],[X(index,2) nav(index,2)],[X(index,3) nav(index,3)])
retroproject_beam (X,nav,index)
[points_out points_in]=retroproject_beam (X,nav,[1:size(index)]);

fzero=zeros(1,size(X',2));
fone=ones(1,size(points_in',2));
fminus=-1*ones(1,size(points_out',2));
X= [X',points_in',points_out'];
y= [fzero,fone,fminus];

 