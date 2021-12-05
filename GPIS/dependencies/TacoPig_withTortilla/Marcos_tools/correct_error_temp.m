object='compresor';
type='laser';
VKernel={'sqexp','Exp','Mat5','Mat3'};
Vpct=[0.00025 0.005 0.01 0.02 0.03 0.05 0.1 0.15 0.2 0.25 0.3]; % percentage of sampling
ck=2; % counter for kernels vector
cp=1;% counter for percentage vector 

% new workspace
pathworkspace=sprintf('./ErrorAnalysis/%s_%s_workspace_EA_corrected.mat',object,type);


for cp=1:size(Vpct,2)
    

%load surfaces?
pathsurface=sprintf('./ErrorAnalysis/%s_%s_surface_%f_%s.mat',object,type,Vpct(cp),VKernel{ck});
load(pathsurface);

% load original points ... to compute errors ...
laser_path=sprintf('./ErrorAnalysis/datasets/originals/tcalib_%s_withnav.csv',object);
%pathnames=sprintf('./ErrorAnalysis/datasets/names/names_%s_%s.mat',type,object);
%load(pathnames);
%[LX Ly Lminx Lmaxx] = load_laser(lasernames{cp});

n=size(LX,2)/3;


% re-compute errors?
pos= find(mf<=meanlimit & mf>=-meanlimit); % augmented 0.04 to -0.04 ... reduced 0.01 to -0.0001
[posx posy posz] = undo_chorizo(pos', res,scale);
LNX=[posx'; posy';posz']; % new laser points
LNy= zeros(1,size(LNX,2)); % new outputs
        
  
 [neighbours distances]=knnsearch(LX(:,1:n)',LNX',1);  % check that ...      
 [neighbours distances_surfaces]=knnsearch(LNX',LX(:,1:n)',1);  % check that ... 

  VSparcity(ck,cp)=Vpct(cp); % The percentage of points ?
  VError(ck,cp)=mean(distances); % the mean of distances
  VStdev(ck,cp)=std(distances);
       
  VSparcity_surface(ck,cp)=Vpct(cp); % The percentage of points ?
  VError_surface(ck,cp)=mean(distances_surfaces); % the mean of distances
  VStdev_surface(ck,cp)=std(distances_surfaces);
  
end
 
% save results ...
%save(pathsurface,'mf','sf','LX','Ly','res','scale','pos','meanlimit');
pathworkspace=sprintf('./ErrorAnalysis/%s_%s_workspace_EA_corrected.mat',object,type);
save(pathworkspace,'VSparcity','VError','VStdev','VSparcity_surface','VError_surface','VStdev_surface');

errorbar(VSparcity_surface, VError_surface,VStdev_surface) % For SqExp


