%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                       Gaussian Process Demo Script
%  Demonstrates GP regression using the taco-pig toolbox on 3-D Data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Add optimization folder
% p = pwd(); slash = p(1);
% addpath(genpath(['..',slash,'optimization']))
addpath(genpath(['optimization']))

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%% 3-D Example%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
close all; clear all; clear functions; clc;
% import tacopig.*;


%% Set up 3-D Data
% Training Data
% groundtruth = @(x,y,z) 5*exp(-(((x).^2)/5+((1-y).^2)/2+((0.5-z).^2)/3))...
%     -4*exp(-(((2-x).^2)/2+((-1-y).^2)/5+((-1.5-z).^2)/2));
% X = (rand(3,50)-0.5)*6;
% y = groundtruth(X(1,:),X(2,:),X(3,:))+1e-2*randn(1,size(X,2));
% 
% [xeva yeva zeva] = meshgrid(-3:0.5:3,-3:0.5:3,-3:0.5:3);
% xstar = [xeva(:)';yeva(:)';zeva(:)'];
% 
% figure; scatter3(X(1,:),X(2,:),X(3,:),40,y,'filled')
% xlabel('x');ylabel('y');zlabel('z'); title('Training Data')
% fprintf('Press any key to continue...\n');
% 

compresorpath='/home/marcos/Desktop/gp_matlab_original/datasets/laser_clean/thin_data/tcalib_compresor_thin0255_withnav.csv';

temp=load(compresorpath);
X=temp(:,5:7);
offset = [0.108987 0.008302 -0.919726]; %% put gps on the correct position
nav= temp(:,11:13);
nav=[(nav(:,1)+offset(1)) (nav(:,2)+offset(2)) (nav(:,3)+offset(3))];

% make that to local coordinates
index= X(:,3)==min(X(:,3));
tt= X(index,:);
X=[(X(:,1)-tt(1)) (X(:,2)-tt(2)) (X(:,3)-tt(3))];
nav=[(nav(:,1)-tt(1)) (nav(:,2)-tt(2)) (nav(:,3)-tt(3))];

%plotting this points in 3D
figure(2)
scatter3(X(:,1),X(:,2),X(:,3),'filled');
%set(gca,'ZDir','reverse')
hold on
scatter3(nav(:,1),nav(:,2),nav(:,3),'filled','r');

%set(gca,'ZDir','reverse') % upsidedown
scatter3(X(index,1),X(index,2),X(index,3),100,'filled','m');
%set(gca,'ZDir','reverse')
hold on
scatter3(nav(index,1),nav(index,2),nav(index,3),100,'filled','m')
 title('Original Raw Data')
%line([X(index,1) nav(index,1)], [X(index,2) nav(index,2)], [X(index,3) nav(index,3)])
%line([X(:,1) nav(:,1)]', [X(:,2) nav(:,2)]', [X(:,3) nav(:,3)]')
% getting boundaries
minx= min(X);   
maxx = max(X);


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
 
res=50;%150; % grid resolution %50
scale.x.min= minx(1);
scale.x.max= maxx(1);
scale.y.min= minx(2);
scale.y.max= maxx(2);
scale.z.min= minx(3);
scale.z.max= maxx(3);

for j=1:res
    for i=1:res
            d=(j-1)* res^2; % delta
            range = (d+(res*(i-1))+1):(res*i)+d;
            xstar(1,range) = linspace(scale.x.min, scale.x.max, res); % in X
            xstar(2,range) = scale.y.min+(i-1)*((scale.y.max-scale.y.min)/res);% in X
            xstar(3,range) = scale.z.min+(j-1)*(scale.z.max/res); % in X
            %xstar(2,range) = 0.1+(i-1)*(0.2/res); % in Y
            %xstar(3,range) = 0+(j-1)*(1.5/res);

    end
end 

%xstar = [xeva(:)';yeva(:)';zeva(:)'];
tsize=int16((size(xstar,2))^(1/3));
xeva=reshape(xstar(1,:),tsize,tsize,tsize);
yeva=reshape(xstar(2,:),tsize,tsize,tsize);
zeva=reshape(xstar(3,:),tsize,tsize,tsize);
% xeva=xstar(1,:)';
% yeva=xstar(2,:)';
% zeva=xstar(3,:)';
pause
%% Set up Gaussian process

% Use a standard GP regression model:
GP = tacopig.gp.Regressor;

% Plug in the data
GP.X = X;
GP.y = y;

% Plug in the components
GP.MeanFn  = tacopig.meanfn.FixedMean(0);
GP.CovFn   = tacopig.covfn.SqExp();%SqExp();
GP.NoiseFn = tacopig.noisefn.Stationary();
GP.objective_function = @tacopig.objectivefn.NLML;
%GP.objective_function = @tacopig.objectivefn.CrossVal;

GP.solver_function = @anneal;

% Initialise the hyperparameters
GP.covpar   = 1*ones(1,GP.CovFn.npar(size(X,1)));
GP.meanpar  = zeros(1,GP.MeanFn.npar(size(X,1)));
GP.noisepar = 1e-3*ones(1,GP.NoiseFn.npar);


%% Learn & Query
tic
GP.learn();
toc
tic
GP.solve();
[mf, vf] = GP.query(xstar,res);
sf  = sqrt(vf);
toc


%% Visualization Marcos
figure
index=find(y==0);
scatter3(X(1,index),X(2,index),X(3,index),100,'k+','LineWidth',2);

pos= find(mf<=0.2 & mf>=-0.2); % augmented 0.04 to -0.04 ... reduced 0.01 to -0.0001
% Convert positions to 3D
[posx posy posz] = undo_chorizo(pos', res,scale);
hold on
scatter3(posx,posy,posz,5,mf(pos)) % colour by mean




%% Visualise GP Outputs

% Build a colormap
cmap = jet(5); 
levels = linspace(-2.5,3.5,5);

% Generate isosurfaces
figure; 
for ii = 1:5
    camlight
    lighting gouraud
    hh(ii) = patch(isosurface(xeva, yeva, zeva, reshape(mf,size(xeva)), levels(ii)));
    set(hh(ii), 'Facecolor', cmap(ii,:), 'Edgecolor', 'none', 'facealpha', (1-(5-abs(levels(ii)))/5));
    axis([-3 3 -3 3 -3 3])
    caxis([-4,5]);
    xlabel('x');ylabel('y');zlabel('z')
    colorbar
end
 title('Predictive Mean')

 hold on
 %mapshow(sq.x,sq.y,'DisplayType','polygon','LineStyle','none')
index=find(y==0);
scatter3(X(1,index),X(2,index),X(3,index),100,'k+','LineWidth',2);


figure;
for i = 1:5
    fstar = GP.sampleprior(xstar);
    clf
    for ii = 1:5
        camlight
        lighting gouraud
        hh(ii) = patch(isosurface(xeva, yeva, zeva, reshape(fstar,size(xeva)), levels(ii)));
        set(hh(ii), 'Facecolor', cmap(ii,:), 'Edgecolor', 'none', 'facealpha', (1-(5-abs(levels(ii)))/5));
        axis([-3 3 -3 3 -3 3])
        caxis([-4,5]);
        xlabel('x');ylabel('y');zlabel('z')
        colorbar
    end
    title('Prior Sample')
    pause(0.5)
end


 
 figure;
for i = 1:5
    fstar = GP.sampleposterior(xstar);
    clf
    for ii = 1:5
        camlight
        lighting gouraud
        hh(ii) = patch(isosurface(xeva, yeva, zeva, reshape(fstar,size(xeva)), levels(ii)));
        set(hh(ii), 'Facecolor', cmap(ii,:), 'Edgecolor', 'none', 'facealpha', (1-(5-abs(levels(ii)))/5));
        axis([-3 3 -3 3 -3 3])
        caxis([-4,5]);
        xlabel('x');ylabel('y');zlabel('z')
        colorbar
    end
    title('Posterior Sample')
    pause(0.5)
end