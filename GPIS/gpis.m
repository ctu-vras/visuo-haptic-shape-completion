% https://github.com/marcospaul/GPIS
function [v, f, vf, xeva, yeva, zeva] = gpis(input_name)
    %% Description
    % This Script loads a 3D point cloud in pcd format with its normals.
    % Then performs a 3D Reconstruction using GPIS.

    %% Add libraries

    % Note to add all depndencies folders or libraries
    addpath(genpath('./dependencies'));

    % Place matlab in the working directory
    %clear all; close all; clc
    disp(input_name)
    D_frompcd=loadpcd(input_name);
    % Format: x y z nx ny nz radius

    %% Prepare Data (Computing Constraints) % Assuming normals are correct ...
    % Parameters can be computed automatically as a function of the size of the object(e.g. 10% of the size defined by a bounding box, see (Wendland, 2002, Surface Reconstructions from unorganized point clouds).
    d=0.01%0.0005; 0.2
    d_pos=0.01%0.0005; 0.2
    d_neg=0.01%0.0005;%0.2
    npar=0.001%0.001;%0.03

    % Grid resolution
    res=100;%150; % grid resolution %50

    %% COmputing inside and outside constraints based on normals
    points_out= [D_frompcd(1,:)+d_neg*D_frompcd(4,:); D_frompcd(2,:)+d_neg*D_frompcd(5,:);D_frompcd(3,:)+d_neg*D_frompcd(6,:)];
    points_in= [D_frompcd(1,:)-d_pos*D_frompcd(4,:) ; D_frompcd(2,:)-d_pos*D_frompcd(5,:); D_frompcd(3,:)-d_pos*D_frompcd(6,:)];

    %% Prepare f(x) as signace distance function
    %fone=ones(1,size(points_in',1))*1;
    %fminus=-1*ones(1,size(points_out',1))*1;
    fone=ones(1,size(points_in',1))*d_pos;
    fminus=-1*ones(1,size(points_out',1))*d_neg;
    X= [D_frompcd(1,:); D_frompcd(2,:) ;D_frompcd(3,:)];
    fzero=zeros(1,size(X',1));


    %% Visualise Object (Cube)
    % Notice that the scale of the Sphere goes from -20 to 20
%     scatter3(D_frompcd(1,:),D_frompcd(2,:),D_frompcd(3,:),3)
%     %ylim(gca,[-1 1]);
%     %xlim(gca,[-1 1]);
%     %zlim(gca,[-1 1]);
%     hold on
%     scatter3(points_out(1,:),points_out(2,:),points_out(3,:),3,'r');
%     scatter3(points_in(1,:),points_in(2,:),points_in(3,:),3,'g');


    % Trainning data
    X= [X,points_in,points_out];
    X= double(X);
    y= [fzero,fone,fminus];

    % Evaluation limits
    minx= min(X')-0.6; % we extend a bit the boundaries of the object to evaluate a little bit further
    maxx = max(X')+0.6; % the 0.6 value can be adjusted dependeing the size of the bounding box, and if for example you are intereseted in regions outside the boundaries of the object modelled by the sensors.

    % filling the query vector :)
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
                xstar(3,range) = scale.z.min+((j)*((scale.z.max-scale.z.min)/res));
        end
    end
    tsize=int16((size(xstar,2))^(1/3));
    xeva=reshape(xstar(1,:),tsize,tsize,tsize);
    yeva=reshape(xstar(2,:),tsize,tsize,tsize);
    zeva=reshape(xstar(3,:),tsize,tsize,tsize);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% ========== GP Setup ===================== %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    GP=tacopig.gp.GP_NoisyInputs;
    % Plug in the data
    GP.X = X;
    GP.y = y;
    %noise
    X_noise = zeros(size(GP.X));
    GP.X_noise = X_noise;
    GP.factorisation = 'SVD'; % 'SVD' or 'CHOL' or 'JITCHOL'
    GP.MeanFn =GP_ConstantMean(mean(GP.y));
    GP.CovFn = GP_ExpCov();

    GP.NoiseFn = GP_ClampNoise(GP_MultiNoise([length(X)]),[1], [npar]); %0.3034 for laser

    GP.solver_function = @fminunc;
    GP.objective_function = @GP_LMLG_FN;
    GP.covpar  = 0.5*ones(1,GP.CovFn.npar(size(GP.X,1)));
    GP.meanpar = zeros(1,GP.MeanFn.npar(size(GP.X,1)));
    GP.noisepar = 1e-3*ones(1,GP.NoiseFn.npar); % not sure if this is OK

    %%%%%%%%%%%%%%

    GP.solver_function = @fminunc;
    GP.objective_function = @GP_LMLG_FN;
    GP.optimset('GradObj', 'off');

    %%%%%%%%%%

    %% Learn & Query
    GP.learn();
    GP.solve();

    %% Solve after learning
    [mf, vf] = GP.query(xstar);
  
    sf  = 2*sqrt(vf);

    %levels = linspace(-2.5,3.5,5);
    levels = [-0.004 -0.003 -0.002 -0.001 0 0.001 0.002 0.003 0.004];

    [f,v,~]=isosurface(xeva, yeva, zeva, reshape(mf,size(xeva)),levels(5),reshape(sf,size(xeva)),'verbose');
%     save('v', 'v');
%     save('f', 'f');
%     save('vf', 'vf');
%     save('mf', 'mf');
%     save('xeva', 'xeva');
%     save('yeva', 'yeva');
%     save('zeva', 'zeva');
end