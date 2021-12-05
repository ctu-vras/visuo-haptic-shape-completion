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
%laser_compresor_path='/home/marcos/Desktop/gp_matlab_original/datasets/laser_clean/thin_data/tcalib_compresor_thin0255_withnav.csv';
%radar_compresor_path='/home/marcos/Desktop/gp_matlab_original/datasets/radar_clean/Spectrum_Distance/thindata/rts_compresor_thin027_withnav.csv';
laser_car_path='/home/marcos/Desktop/gp_matlab_original/datasets/laser_clean/thin_data/tcalib_car_clean_thin00829_withnav.csv';
radar_car_path='/home/marcos/Desktop/gp_matlab_original/datasets/radar_clean/Spectrum_Distance/thindata/rts_car_thin01_withnav.csv';

%% Set up 3-D Data
[LX Ly lminx lmaxx ref_laser] = load_laser(laser_car_path);
[RX Ry RI rminx rmaxx ref_radar] = load_radar(radar_car_path,laser_car_path);

minx= min(rminx,lminx);
maxx= max(rmaxx,lmaxx);


%line([X(index,1) nav(index,1)], [X(index,2) nav(index,2)], [X(index,3) nav(index,3)])
%line([X(:,1) nav(:,1)]', [X(:,2) nav(:,2)]', [X(:,3) nav(:,3)]')
% getting boundaries
%minx= min(LX);   
%maxx = max(LX);

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
%% Set up Gaussian process for LASER

% Use a standard GP regression model:
LGP = tacopig.gp.Regressor;

% Plug in the data
LGP.X = LX;
LGP.y = Ly;

% Plug in the components
LGP.MeanFn  = tacopig.meanfn.FixedMean(0);
LGP.CovFn   = tacopig.covfn.SqExp();%SqExp();
LGP.NoiseFn = tacopig.noisefn.Stationary();
GP.objective_function = @tacopig.objectivefn.NLML;
%LGP.objective_function = @tacopig.objectivefn.CrossVal;

LGP.solver_function = @anneal;

% Initialise the hyperparameters
LGP.covpar   = 1*ones(1,LGP.CovFn.npar(size(LX,1)));
LGP.meanpar  = zeros(1,LGP.MeanFn.npar(size(LX,1)));
LGP.noisepar = 1e-3*ones(1,LGP.NoiseFn.npar);


% Learn & Query: Laser
tic
LGP.learn();
toc

tic
LGP.solve();
[Lmf, Lvf] = LGP.query(xstar,res);
Lsf  = sqrt(Lvf);
toc


% Visualization Marcos Laser
figure
index=find(Ly==0);
scatter3(LX(1,index),LX(2,index),LX(3,index),100,'k+','LineWidth',2);

pos= find(Lmf<=0 & Lmf>=-0.1); % augmented 0.04 to -0.04 ... reduced 0.01 to -0.0001
% Convert positions to 3D
[posx posy posz] = undo_chorizo(pos', res,scale);
LNX=[posx'; posy';posz'];
LNy= zeros(1,size(LNX,2));

hold on
scatter3(posx,posy,posz,5,Lmf(pos)) % colour by mean



%this is just for test 
npos=find(Lsf(pos)<=0.4);
scatter3(posx(npos),posy(npos),posz(npos),5,Lsf(pos(npos))) % colour by mean

% Write laser
basename='car_thin00829_sqexp'
type='laser';

t= [posx(npos) posy(npos) posz(npos) abs(Lsf(pos(npos)))'];
index=find(Ly==0);
tt= [LX(1,index); LX(2,index); LX(3,index)]; % original sampled?
%pos= find(mf<=0.2 & mf>=-0.2); % augmented 0.04 to -0.04 ... reduced 0.01 to -0.0001
%[posx posy posz] = undo_chorizo(pos, res,scale);
ttt=[posx(npos)'; posy(npos)'; posz(npos)'; Lmf(pos(npos)); abs(Lsf(pos(npos)))];

LNX=[posx(npos)'; posy(npos)';posz(npos)'];
LNy= zeros(1,size(LNX,2));

save('fusion_compresor_workspace.mat','Lmf','Lsf','pos','npos','res','scale','LX','Ly','pos','npos','LNX','LNy')
scatter3(LNX(1,:)',LNX(2,:)',LNX(3,:)',5) % colour by mean

%write_file(basename,type,t,tt);

%% Now you have the two models ... laser and radar time to fuse them ... uisng raw data

%[RX Ry RI rminx rmaxx] = load_radar(radar_car_path,laser_car_path);


% do radar constrains :) ...
nr=size(RX,2)/3;

NX=[RX(:,1:nr) LX];
Ny=[Ry(:,1:nr) Ly];


RGP = tacopig.gp.Regressor;
RGP.X=NX;
RGP.y=Ny;


%RGP.X = RX;
%RGP.y = Ry;

% Plug in the components
RGP.MeanFn  = tacopig.meanfn.FixedMean(0);
RGP.CovFn   = tacopig.covfn.SqExp();%SqExp();
RGP.NoiseFn = tacopig.noisefn.Stationary();
RGP.objective_function = @tacopig.objectivefn.NLML;
%LGP.objective_function = @tacopig.objectivefn.CrossVal;

RGP.solver_function = @anneal;

% Initialise the hyperparameters
RGP.covpar   = 1*ones(1,RGP.CovFn.npar(size(RX,1)));
RGP.meanpar  = zeros(1,RGP.MeanFn.npar(size(RX,1)));
RGP.noisepar = [1e-3*ones(1,RGP.NoiseFn.npar)  20e-3*ones(1,RGP.NoiseFn.npar)];


% Learn & Query: Laser
tic
RGP.learn();
toc

tic
RGP.solve();
[Rmf, Rvf] = RGP.query(xstar,res);
Rsf  = sqrt(Rvf);
toc

tic
RGP.solve();
[Rmf, Rvf] = RGP.query(xstar,res);
Rsf  = sqrt(Rvf);
toc

meanlimit=+0.05;
pos= find(Rmf<=0.0 & Rmf>=-0.07); % au
npos= find(Rsf(pos)<=0.25); % au

[posx posy posz] = undo_chorizo(pos', res,scale);
scatter3(posx(npos),posy(npos),posz(npos),5,Rsf(pos(npos))) % colour by mean
%scatter3(posx,posy,posz,5,Rsf(pos)) % colour by mean

%tt= [LNX(1,1:sso)'+delta(1) LNX(2,1:sso)'+delta(2) LNX(3,1:sso)'+delta(3) Lsf(pos(npos))'];
%write_ply('surfacelaser.ply',tt');
[LoX Loy tmin tmax ref_original]=load_laser('ErrorAnalysis/datasets/originals/tcalib_car_withnav.csv');
delta=ref_laser-ref_original;

rt= [posx(npos)+delta(1) posy(npos)+delta(2) posz(npos)+delta(3) Rsf(pos(npos))'];
write_ply('surfaceradar_clean.ply',rt');


[RoX Roy RI rtmin rtmax Rref_original]=load_radar('ErrorAnalysis/datasets/originals/rts_car_withnav.csv','ErrorAnalysis/datasets/originals/tcalib_car_withnav.csv');
rfo=size(RoX,2)/3;
deltar=Rref_original-ref_original;
rtt=[RoX(1,1:rso)' RoX(2,1:rso)' RoX(3,1:rso)' RI];
write_ply('fullradarorginal.ply',rtt');

%Sampled file

[RsX Rsy RsI Rsminx Rsmaxx Rsref_radar] = load_radar(radar_car_path,laser_car_path);
rso=size(RsX,2)/3;
deltar=Rsref_radar - ref_original;
rttt=[RsX(1,1:rso)'+deltar(1) RsX(2,1:rso)'+deltar(2) RsX(3,1:rso)'+deltar(3) RsI];
write_ply('sampledradarorginal.ply',rttt');












LR=[posx(npos)'; posy(npos)';posz(npos)']; % new laser-radar points

figure
index=find(Ly==0);
scatter3(LX(1,index),LX(2,index),LX(3,index),100,'k+','LineWidth',2);
scatter3(LR(1,:),LR(2,:),LR(3,:));

%[LoX Loy tmin tmax ref_original]=load_laser('ErrorAnalysis/datasets/originals/tcalib_car_withnav.csv');

[neighbours distances_L]=knnsearch(LNX',LoX',1);  % check that ...      
[neighbours distances_LR]=knnsearch(LR',LoX',1);  % check that ...      

Lmean=mean(distances_L)
LRmean=mean(distances_LR)

Lstd=std(distances_L);
LRstd=std(distances_LR);

% Normals ...


dt=DelaunayTri(LNX(1,:)',LNX(2,:)',LNX(3,:)');
[tri Xb] = freeBoundary(dt);
tr = TriRep(tri, Xb);
P = incenters(tr);
fn = faceNormals(tr);

%trisurf(tri,Xb(:,1),Xb(:,2),Xb(:,3), ...
%'FaceColor', 'cyan', 'faceAlpha', 0.8);
scatter3(LNX(1,:),LNX(2,:),LNX(3,:),'k');
hold on

%quiver3(P(:,1),P(:,2),P(:,3), ...
%fn(:,1),fn(:,2),fn(:,3),1.5, 'color','r');

pconstrains=P-(fn*2/10);  % check the distances of the constrains
nconstrains=P+(fn*5/10);
scatter3(pconstrains(:,1),pconstrains(:,2),pconstrains(:,3),'+b');
scatter3(nconstrains(:,1),nconstrains(:,2),nconstrains(:,3),'xr');


% Now should we trust all the normals equally?
% lets plot the minimun distances to a number of neighbours :)
%for i=1:size(P,1)
[neighbours distances_L]=knnsearch(P,P,10);  % check that ...      
scatter3(P(:,1),P(:,2),P(:,3),5,mean(distances_L'));

[f,x]=hist(mean(distances_L'))
%g=1/sqrt(2*pi)*exp(-0.5*x.^2);%# pdf of the normal distribution

%#METHOD 1: DIVIDE BY SUM
%figure(1)
%bar(x,f/sum(f));hold on
%plot(x,g,'r');hold off

%#METHOD 2: DIVIDE BY AREA
%figure(2)
%bar(x,f/trapz(x,f));hold on
%plot(x,g,'r');hold off
% translate normals to points in space
mean_distance=mean(mean(distances_L'));
sigma=std(mean(distances_L'));
distance_th=mean_distance + 3*sigma;
index=find(mean(distances_L')<=distance_th);
scatter3(P(index,1),P(index,2),P(index,3),5,mean(distances_L(index,:)'));
figure;scatter3(P(:,1),P(:,2),P(:,3),5,mean(distances_L(:,:)'));

pconstrains=pconstrains(index,:);
ncontrains=nconstrains(index,:);


%% Now you have the two models ... using GP from laser and raw radar
[RX Ry RI rminx rmaxx ref_newradar] = load_radar(radar_car_path,laser_car_path);
% do radar constrains :) ...
nr=size(RX,2)/3;
nlgp=size(LNX,2);
%% Resampling ...
sampling_factor=2*nr/nlgp; %twice the number of points from the radar :)

% creating a sampled file
surface_filename=('lasersurface_temp.csv');
formatSpec = '%f,%f,%f\n';
fileID = fopen(surface_filename,'w');
fprintf(fileID,formatSpec,LNX);

command=sprintf('cat %s | csv-thin %f > sampled_temp.csv', surface_filename,sampling_factor);
system(command)

LSS=load('sampled_temp.csv');% Laser surface sampled
Lyy=zeros(1,size(LSS,1));

% Now how many constrains?

index=randsample([1:size(pconstrains,1)],nr);

%using artificial constrains from surface
%NX=[RX(:,1:nr) LSS' pconstrains(index,:)' nconstrains(index,:)'];
%Ny=[Ry(:,1:nr) Ly   ones(size(index,2),1)'       -ones(size(index,2),1)' ];

%reload Laser GP ...
nls=size(LX,2)/3;
NX=[RX(:,1:nr) LSS' LX(:,nls+1:nls*2) LX(:,nls*2+1:nls*3)];
Ny=[Ry(:,1:nr) Lyy   Ly(:,nls+1:nls*2) Ly(:,nls*2+1:nls*3)];

LsRGP = tacopig.gp.Regressor;
LsRGP.X=NX;
LsRGP.y=Ny;


%RGP.X = RX;
%RGP.y = Ry;

% Plug in the components
LsRGP.MeanFn  = tacopig.meanfn.FixedMean(0);
LsRGP.CovFn   = tacopig.covfn.SqExp();%SqExp();
LsRGP.NoiseFn = tacopig.noisefn.Stationary();
LsRGP.objective_function = @tacopig.objectivefn.NLML;
%LGP.objective_function = @tacopig.objectivefn.CrossVal;

LsRGP.solver_function = @anneal;

% Initialise the hyperparameters
LsRGP.covpar   = 1*ones(1,LsRGP.CovFn.npar(size(NX,1)));
LsRGP.meanpar  = zeros(1,LsRGP.MeanFn.npar(size(NX,1)));
LsRGP.noisepar = [1e-3*ones(1,LsRGP.NoiseFn.npar)  20e-3*ones(1,LsRGP.NoiseFn.npar)];


% Learn & Query: Laser
tic
LsRGP.learn();
toc

%% add the xstar here ...
tic
LsRGP.solve();
[LsRmf, LsRvf] = LsRGP.query(xstar,res);
LsRsf  = sqrt(LsRvf);
toc


pos= find(LsRmf<=0.2 & LsRmf>=-0.2); % augmented 0.04 to -0.04 ... reduced 0.01 to -0.0001
[posx posy posz] = undo_chorizo(pos', res,scale);
npos=find(LsRsf(pos)<0.25);

% correction factor
delta=ref_laser-ref_original;

scatter3(posx(npos)+delta(1),posy(npos)+delta(2),posz(npos)+delta(3),5,LsRsf(pos(npos))) % colour by mean
hold on
scatter3(LNX(1,:),LNX(2,:),LNX(3,:),'k');
scatter3(LoX(1,:),LoX(2,:),LoX(3,:),'k');

t= [posx(npos)+delta(1) posy(npos)+delta(2) posz(npos)+delta(3) LsRsf(pos(npos))'];

write_ply('surfacelgprr.ply',t');



%load laser surface

[posx posy posz] = undo_chorizo(pos', res,scale);
sso=size(LNX,2)
tt= [LNX(1,1:sso)'+delta(1) LNX(2,1:sso)'+delta(2) LNX(3,1:sso)'+delta(3) Lsf(pos(npos))'];
write_ply('surfacelaser.ply',tt');

tt_notclean= [posx+delta(1) posy+delta(2) posz+delta(3) Lsf(pos)'];
write_ply('surfacelaser_notclean.ply',tt_notclean');





[LoX Loy tmin tmax ref_original]=load_laser('ErrorAnalysis/datasets/originals/tcalib_car_withnav.csv');
so=size(LoX,2)/3;
ttt=[LoX(1,1:so)' LoX(2,1:so)' LoX(3,1:so)' zeros(1,so)'];

write_ply('fulllaserorginal.ply',ttt');



%Sampled file

[LX Ly lminx lmaxx ref_laser] = load_laser(laser_car_path);
ss=size(LX,2)/3;
tttt=[LX(1,1:ss)'+delta(1) LX(2,1:ss)'+delta(2) LX(3,1:ss)'+delta(3) zeros(1,ss)'];
write_ply('sampledlaserorginal.ply',tttt');



%% plotting

% plot surface laser ...
scatter3(LNX(1,:),LNX(2,:),LNX(3,:),5,'b') % not colour
hold on
nr=size(RX,2)/3;
scatter3(RX(1,nr+1:2*nr),RX(2,nr+1:2*nr),RX(3,nr+1:2*nr),5,'r') % not colour
scatter3(RX(1,2*nr+1:3*nr),RX(2,2*nr+1:3*nr),RX(3,2*nr+1:3*nr),5,'g') % not colour



% plot artificial radar constrains .. 
% plot 

pos= find(Lmf<=0.2 & Lmf>=-0.2); % augmented 0.04 to -0.04 ... reduced 0.01 to -0.0001
% Convert positions to 3D
[posx posy posz] = undo_chorizo(pos', res,scale);
RNX=[posx'; posy';posz'];
RNy= zeros(1,size(LNX,2));

hold on
scatter3(posx,posy,posz,5,Rvf(pos)) % colour by mean










%% Set up GP for Radar ... 
[RX Ry RI rminx rmaxx] = load_radar(radar_car_path,laser_car_path);
RGP = tacopig.gp.Regressor;

% Plug in the data
RGP.X = RX;
RGP.y = Ry;

% Plug in the components
RGP.MeanFn  = tacopig.meanfn.FixedMean(0);
RGP.CovFn   = tacopig.covfn.SqExp();%SqExp();
RGP.NoiseFn = tacopig.noisefn.Stationary();
RGP.objective_function = @tacopig.objectivefn.NLML;
%LGP.objective_function = @tacopig.objectivefn.CrossVal;

RGP.solver_function = @anneal;

% Initialise the hyperparameters
RGP.covpar   = 1*ones(1,RGP.CovFn.npar(size(RX,1)));
RGP.meanpar  = zeros(1,RGP.MeanFn.npar(size(RX,1)));
RGP.noisepar = 1e-3*ones(1,RGP.NoiseFn.npar);


% Learn & Query: Laser
tic
RGP.learn();
toc

tic
RGP.solve();
[Rmf, Rvf] = RGP.query(xstar,res);
Rsf  = sqrt(Rvf);
toc


% Visualization Marcos Laser
figure
index=find(Ry==0);
scatter3(RX(1,index),RX(2,index),RX(3,index),100,'k+','LineWidth',2);

pos= find(Rmf<=0.2 & Rmf>=-0.2); % augmented 0.04 to -0.04 ... reduced 0.01 to -0.0001
% Convert positions to 3D
[posx posy posz] = undo_chorizo(pos', res,scale);
hold on
scatter3(posx,posy,posz,5,Rmf(pos)) % colour by mean
RNX=[posx'; posy';posz'];
RNy= zeros(1,size(RNX,2));

% Writing files ...
pos= find(Rmf<=0.2 & Rmf>=-0.2); % augmented 0.04 to -0.04 ... reduced 0.01 to -0.0001
[posx posy posz] = undo_chorizo(pos', res,scale);
t= [posx posy posz abs(Rsf(pos))'];
index=find(Ry==0);
tt= [RX(1,index); RX(2, index); RX(3,index); RI(index)'];
formatSpec = '%f %f %f %f %f\n';
pos= find(Rmf<=0.4 & Rmf>=-0.4); % augmented 0.04 to -0.04 ... reduced 0.01 to -0.0001
[posx posy posz] = undo_chorizo(pos', res,scale);
ttt=[posx'; posy'; posz'; Rmf(pos); abs(Rsf(pos));];
basename='car_thin00829_sqexp';
type='radar';
write_file(basename,type,t,tt);







