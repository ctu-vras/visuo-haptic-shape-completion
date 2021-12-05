%% Generating Sampled Files ...
%clear all; clc;
%object='fence';
%type='laser';
% note: we need to be at ./Marcos_tools to use this function
function generate_gpnames(object,type,Vpct)
%laser_path='./ErrorAnalysis/datasets/originals/tcalib_compresor_withnav.csv';
%res=150;
ck=1; % counter for kernels vector
cp=1;% counter for percentage vector 
%j=1;
%tempfile='temp_downsample.csv';
VKernel={'sqexp','Exp','Mat5','Mat3'};


lasernames={{}};

for ck=1:size(VKernel,2)
    for (cp=1:size(Vpct,2))
        pct=Vpct(cp); % percentage of data points where 1 is 100%
        file=sprintf('./ErrorAnalysis/%s_%s_surface_%f_%s.mat',object,type,Vpct(cp),VKernel{ck});
        surfacenames{cp,ck}=file;
        %save(pathsurface,Lvf,Lmf);
        fprintf('File: %s processed DONE!\n',file)
    end
end



filenames=sprintf('./ErrorAnalysis/datasets/names/gpsurfaces/names_GPsurface_%s_%s.mat',type,object);
save(filenames,'surfacenames');
% test with heads
command=sprintf('cat %s | head ',surfacenames{1});
system(command)

% accessing by lasernames{1} e.g.