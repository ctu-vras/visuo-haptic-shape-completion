%% Generating Sampled Files ...
%clear all; clc;
%object='fence';
%type='laser';
% note: we need to be at ./Marcos_tools to use this function
function setdownsampledfiles_mod(object,type,Vpct)
%laser_path='./ErrorAnalysis/datasets/originals/tcalib_compresor_withnav.csv';
res=150;
ck=1; % counter for kernels vector
cp=1;% counter for percentage vector 
%j=1;
tempfile='temp_downsample.csv';
VKernel={'sqexp','Exp','Mat5','Mat3'};

lasernames={};


for (cp=1:size(Vpct,2))
    pct=Vpct(cp); % percentage of data points where 1 is 100%
    filesampled=sprintf('./ErrorAnalysis/datasets/downsampled/%s_%s/%s_%s_sampled_%f.csv',type,object,type,object,Vpct(cp));
    lasernames{cp}=filesampled;
    %save(pathsurface,Lvf,Lmf);
    fprintf('File: %s processed DONE!\n',filesampled)
    
end

filenames=sprintf('./ErrorAnalysis/datasets/names/names_%s_%s.mat',type,object);
save(filenames,'lasernames');
% test with heads
command=sprintf('cat %s | head ',lasernames{1});
system(command)

% accessing by lasernames{1} e.g.