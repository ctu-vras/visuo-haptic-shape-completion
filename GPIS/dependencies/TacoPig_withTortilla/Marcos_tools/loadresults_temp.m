clear all; clc;
type='laser';
object='compresor';
cp=4;
ck=1;
VKernel={'sqexp','Exp','Mat5','Mat3'};
Vpct=[0.00025 0.005 0.01 0.02 0.03 0.05 0.1 0.15 0.2 0.25 0.3 0.4 0.5]; % percentage of sampling


laser_path=sprintf('./ErrorAnalysis/datasets/originals/tcalib_%s_withnav.csv',object);

pathnames=sprintf('./ErrorAnalysis/datasets/names/names_%s_%s.mat',type,object);
load(pathnames);

[X y minx maxx] = load_laser(laser_path); % load original
%[LX Ly Lminx Lmaxx] = load_laser(lasernames{cp}); % load sampled data
% load surface
pathsurface=sprintf('./ErrorAnalysis/%s_%s_surface_%f_%s.mat',object,type,Vpct(cp),VKernel{ck});
load(pathsurface);
%plot surface
[posx posy posz] = undo_chorizo(pos', res,scale);
n=size(X,2)/3;
[neighbours distances]=knnsearch(X(:,1:n)',LX',1);
mean(distances)
scatter3(posx,posy,posz,5,mf) % colour by mean