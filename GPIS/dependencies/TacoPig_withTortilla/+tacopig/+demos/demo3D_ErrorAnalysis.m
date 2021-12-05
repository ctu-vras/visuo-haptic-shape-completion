%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                       Gaussian Process Demo 3D Error Analysis
% Plots errors graphics in IRO 2013 paper
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Add optimization folder
% p = pwd(); slash = p(1);
% addpath(genpath(['..',slash,'optimization']))
type='laser';
object='compresor';
function Error_Analysis(type,object);

addpath(genpath(['optimization']))

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%% 3-D Example%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%close all; clear all; clear functions; clc;
% Original Laser File (Unsampled ... Add Nav??)
%we need: 
% laser_path
% type
% object

%tempfile='temp_downsample.csv';


VKernel={'sqexp','Exp','Mat5','Mat3'};
Vpct=[0.01 0.02 0.03 0.05 0.1 0.15 0.2 0.25 0.3 0.4 0.5]; % percentage of sampling

res=150;
ck=1; % counter for kernels vector
cp=1;% counter for percentage vector 
%j=1;


if (strcmp(type,'radar'))
    laser_path=sprintf('./ErrorAnalysis/datasets/originals/rts_%s_withnav.csv',object);
else
    laser_path=sprintf('./ErrorAnalysis/datasets/originals/tcalib_%s_withnav.csv',object);
end
% Load Names of files to be processed
pathnames=sprintf('./ErrorAnalysis/datasets/names/names_%s_%s.mat',type,object);
load(pathnames);

% For Error Graph
VSparcity=[]; % The percentage of points ?
VError=[]; % the mean of distances
VStdev=[];

%%Random Sampling ???
[X y minx maxx] = load_laser(laser_path);
n=size(X,2)/3;
%k=round(n*pct);
%index=randsample(n,k);

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
close all;
for (ck=1:size(VKernel,2))
    {
    for (cp=1:size(Vpct,2))


        pct=Vpct(cp); % percentage of data points where 1 is 100%
        %command=sprintf('cat %s | csv-thin %s > %s',laser_path,pct,tempfile);
        %system(command)
        [LX Ly Lminx Lmaxx] = load_laser(lasernames{cp});
        close all;

        %% Set up Gaussian process for LASER

        % Use a standard GP regression model:
        LGP = tacopig.gp.Regressor;

        % Plug in the data
        LGP.X = LX;
        LGP.y = Ly;

        % Plug in the components
        LGP.MeanFn  = tacopig.meanfn.FixedMean(0);

        if strcmp(VKernel(ck),'sqexp')
            LGP.CovFn   = tacopig.covfn.SqExp();%SqExp();
        else
            display('Cov Function error ... using sqExp by default!!!')
            LGP.CovFn   = tacopig.covfn.SqExp();%SqExp();
        end


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
        %figure
        %index=find(Ly==0);
        %scatter3(LX(1,index),LX(2,index),LX(3,index),100,'k+','LineWidth',2);

        range=max(Lmf) -min(Lmf);
        if (range ==0)
            Error('Error:something is wrong, not enought points')
        end


            meanlimit=+0.2;
            pos= find(Lmf<=meanlimit & Lmf>=-meanlimit); % augmented 0.04 to -0.04 ... reduced 0.01 to -0.0001
            while (size(pos,2)>=150000 | size(pos,2)<=5000)  
                if (size(pos,2)>=150000)
                    % reduce size
                    meanlimit=meanlimit/2;
                    %pos= find(Lmf<=meanlimit & Lmf>=-meanlimit);
                elseif size(pos,2)<=5000
                    meanlimit=meanlimit*2;
                end
                  pos= find(Lmf<=meanlimit & Lmf>=-meanlimit); % augmented 0.04 to -0.04 ... reduced 0.01 to -0.0001
            end

        [posx posy posz] = undo_chorizo(pos', res,scale);
        LNX=[posx'; posy';posz']; % new laser points
        LNy= zeros(1,size(LNX,2)); % new outputs

        [neighbours distances]=knnsearch(X(:,1:n)',LNX',1);        

        VSparcity(ck,cp)=pct; % The percentage of points ?
        VError(ck,cp)=mean(distances); % the mean of distances
        VStdev(ck,cp)=std(distances);
        %% Saving Data
        % workspace
        pathworkspace=sprintf('./ErrorAnalysis/%s_%s_workspace_EA.mat',object,type);
        save(pathworkspace,'VSparcity','VError','VStdev');
        % surface
        pathsurface=sprintf('./ErrorAnalysis/%s_%s_surface_%f_%s.mat',object,type,Vpct(cp),cell2num(VKernel(ck)));
        sf=Lsf(pos);
        mf=Lmf(pos);
        save(pathsurface,'mf','sf','LX','Ly','res','scale','pos');


    fprintf('%f Percentage has been processed, using %s kernel\n',VSparcity(cp),cell2num(VKernel(ck)))
    end
end

% For Error Graph lets stock this values :)

%errorbar(VSparcity, VError,VStdev) % For SqExp
    
%% Saving Data
% workspace
% pathworkspace=sprintf('/ErrorAnalysis/%s_%s_workspace_EA',object,type);
% save(pathworkspace,'Vsparcity','Verror','Vstdev');
% % surface
% pathsurface=sprintf('/ErrorAnalysis/%s_%s_surface_%f_%s',object,type,Vpct(cp),Vkernel(ck));
% save(pathsurface,Lvf,Lmf);

