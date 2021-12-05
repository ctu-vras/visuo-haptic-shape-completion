%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%                       Implicit Surface Demo
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all; clear all; clear functions; clc;
% Add working path
p = pwd;
root = strfind(p, 'geotherML')+9;
slash = p(root);     root = p(1:root);
Libroot = [root, 'lib', slash];
addpath(genpath(Libroot))

%load data
X = [0.1545, 0.2850,0.4827, 0.6459, 0.8455, 0.8954, 0.7514, 0.5749, 0.4789, 0.3560,...
    0.2006, 0.0835, 0.1238, 0.4616, 0.4463, 0.9549, 0.8973, 0.3311, 0.0144, 0.0566;...    
    0.3248, 0.2202, 0.2737, 0.2762, 0.3175, 0.5097, 0.6873, 0.5633, 0.4927, 0.6119,...    
    0.6509, 0.5365, 0.3905, 0.3783, 0.8285, 0.6265, 0.0864, 0.0304, 0.1691, 0.8285];
y = [0 0 0 0 0 0 0 0 0 0 0 0 0 1 -1 -1 -1 -1 -1 -1];
   
X_noise = zeros(size(X));
n = size(X,2);

%Create query data
[xeva yeva] = meshgrid(0:0.02:1, 0:0.02:1);
xstar = [xeva(:)';yeva(:)']


%% Configure the standard GP model
 GP = GP_NoisyInputs(); 
%GP = GP_STD(); 
GP.X = X;
GP.y = y;
   GP.X_noise = X_noise;
GP.factorisation = 'SVD'; % 'SVD' or 'CHOL' or 'JITCHOL'

%% Mean Functions:
    GP.MeanFn =GP_ConstantMean(-5);

%% Covariance Functions:
    GP.CovFn = GP_Remap(GP_SqExpCov(), [1 1 2]);  %Isotropic Squared Exponential


%% Noise Functions:

%     GP.NoiseFn = GP_ClampNoise(GP_StatNoise(), 1, 1e-1); 
    %GP.NoiseFn = GP_ClampNoise(GP_MultiNoise([13 1 6]),[1:3], [1e-3 1e-3 1e-3]); 
GP.NoiseFn = GP_ClampNoise(GP_MultiNoise([13 7]),[1:2], [1e-3 1e-3]); 


%% Optimisation Parameters
    GP.solver_function = @fminunc; 
    GP.objective_function = @GP_LMLG_FN;
    GP.optimset('gradobj', 'off');


%% Seed initial hyperparameters with auto-sized vectors
GP.covpar  = 0.2*ones(1,GP.CovFn.npar(size(X,1)));
GP.meanpar = zeros(1,GP.MeanFn.npar(size(X,1)));


%% Solve before learning
GP.learn();
GP.solve(); 
[mf, vf] = GP.query(xstar);
sf  = sqrt(vf);
Imp = reshape(mf,size(xeva));

figure(1);subplot(1,3,1);
contour(xeva,yeva,Imp,[0 0],'LineWidth',2)
hold on
plot(X(1,1:13),X(2,1:13),'k+','MarkerSize',10,'LineWidth',4)
title('Predictied Surface')

figure(1); subplot(1,3,2);
surf(xeva,yeva,reshape(mf,size(xeva)))
title('Underlying GP Posterior Mean')

    subplot(1,3,3);hold on
    for i = 1:5
        fstar = GP.sampleposterior(xstar);
        ImpSamp = reshape(fstar,size(xeva));
        contour(xeva,yeva,ImpSamp,[0 0],'color', rand(3,1));
    end
    plot(X(1,1:13),X(2,1:13),'k+','MarkerSize',10,'LineWidth',4)
    title('Samples from the posterior')

disp('Press any key to add an additional surface point')
pause

% %% Add more support
% figure(2); 
% contour(xeva,yeva,Imp,[0 0],'LineWidth',2)
% hold on
% plot(X(1,1:13),X(2,1:13),'k+','MarkerSize',10,'LineWidth',4)
% title('Click to add a surface point')
% 
% counter = 1;
% for i=1:3
% 
%     disp('Press any key to add an additional surface point')
%     [x y] =ginput(1)
%     GP.NoiseFn = GP_ClampNoise(GP_MultiNoise([13 1 6 counter]),[1:3], [1e-3 1e-3 1e-3]); 
%     GP.noisepar = [0.03]; %
%     GP.X = [GP.X [x;y]];
%     GP.y = [GP.y 0];
% %     GP.X_noise = [GP.X_noise [0.03;0.03]]
%     counter = counter+1;
% 
%     GP.learn();
%     GP.solve(); 
%     [mf, vf] = GP.query(xstar);
%     figure; hold on;
%     for i = 1:5
%         fstar = GP.sampleposterior(xstar);
%         Imp = reshape(fstar,size(xeva));
%         contour(xeva,yeva,Imp,[0 0],'color', rand(3,1));
%     end
%     contour(xeva,yeva,Imp,[0 0],'LineWidth',2)
%     plot(X(1,1:13),X(2,1:13),'k+','MarkerSize',10,'LineWidth',4)
%     plot(GP.X(1,14),GP.X(2,14),'rx')
%     plot(GP.X(1,15:20),GP.X(2,15:20),'bx')
%     plot(GP.X(1,21:end),GP.X(2,21:end),'kx')
% 
% end
