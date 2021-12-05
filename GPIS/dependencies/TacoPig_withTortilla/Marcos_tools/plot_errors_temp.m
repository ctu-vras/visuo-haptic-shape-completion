
% Placing Errors Together?

 Ecompresor=load('/home/marcos/Programming/ML/TacoPig/Marcos_tools/ErrorAnalysis/compresor_laser_workspace_EA.mat')
 Ecompresor_inc=load('/home/marcos/Programming/ML/TacoPig/Marcos_tools/ErrorAnalysis/compresor_laser_workspace_EA_backup.mat');

 Etemp=Ecompresor;
 
 Etemp.VError_surface(1,:)=Ecompresor_inc.VError_surface(1,:);
 Etemp.VSparcity(1,:)=Ecompresor_inc.VSparcity(1,:);
 Etemp.VStdev_surface(1,:)=Ecompresor_inc.VStdev_surface(1,:);
 
 Col={'-ok' '-ob' '-oc' '-or'};
 N= size(Etemp.VError_surface,1); %4

 N=1;
 
%Then loop:

for n =1:N
   errorbar(Etemp.VSparcity(n,:),Etemp.VStdev_surface(n,:),Etemp.VError_surface(n,:),Col{n});
   hold on
end
legend('SqExp','Exp','Mat5','Mat3');
title('Error Analysis: Compressor');
xlabel('Concentration of points');
ylabel('Error')