%% Generating Sampled Files ...
%clear all; clc;
%object='fence';
%type='laser';
% note: we need to be at ./Marcos_tools to use this function
function setdownsampledfiles_mod(object,type,Vpct)
%laser_path='./ErrorAnalysis/datasets/originals/tcalib_compresor_withnav.csv';
if (strcmp(type,'radar'))
    laser_path=sprintf('./ErrorAnalysis/datasets/originals/rts_%s_withnav.csv',object);
else
    laser_path=sprintf('./ErrorAnalysis/datasets/originals/tcalib_%s_withnav.csv',object);
end


%Vpct=[0.00025 0.005 0.01 0.02 0.03 0.05 0.1 0.15 0.2 0.25 0.3 0.4 0.5]; % percentage of sampling
res=150;
ck=1; % counter for kernels vector
cp=1;% counter for percentage vector 
%j=1;
tempfile='temp_downsample.csv';
VKernel={'sqexp','Exp','Mat5','Mat3'};

lasernames={};

command=sprintf('mkdir ./ErrorAnalysis/datasets/downsampled/%s_%s',type,object);
system(command)

for (cp=1:size(Vpct,2))


    pct=Vpct(cp); % percentage of data points where 1 is 100%
    
    filesampled=sprintf('./ErrorAnalysis/datasets/downsampled/%s_%s/%s_%s_sampled_%f.csv',type,object,type,object,Vpct(cp));
    lasernames{cp}=filesampled;
    %save(pathsurface,Lvf,Lmf);
    
    command=sprintf('cat %s | csv-thin %s > %s',laser_path,pct,filesampled);
    system(command)
    fprintf('File: %s processed DONE!\n',filesampled)
    
end

filenames=sprintf('./ErrorAnalysis/datasets/names/names_%s_%s.mat',type,object);
save(filenames,'lasernames');
% test with heads
command=sprintf('cat %s | head ',lasernames{1});
system(command)

% accessing by lasernames{1} e.g.