% Script to run in server
/usr/local/MATLAB/R2012a/bin/matlab -r "addpath(genpath('~/TacoPig')); cd ~/TacoPig/Marcos_tools/; type='laser'; object='fence'; demo3D_ErrorAnalysis(type,object); ; exit" -nojvm -nodisplay -nosplash

% server UV
"addpath(genpath('~/Matlab/code/TacoPig')); cd ~/Matlab/code/TacoPig/Marcos_tools/; type='laser'; object='trailer'; demo3D_ErrorAnalysis_temp(type,object); ; exit" -nojvm -nodisplay -nosplash