Vpct1=[0.00025 0.005 0.01 0.02 0.03 0.04 0.05 0.06 0.07 0.08 0.1]; % car/trailerpercentage of sampling
Vpct2=[0.00025 0.005 0.01 0.02 0.03 0.05 0.07 0.085 0.1 0.125 0.15]; % wall percentage of sampling
Vpct3=[0.005 0.01 0.02 0.03 0.05 0.1 0.15 0.2 0.25 0.3 0.4]; % fence percentage of sampling

type='laser';
object='car';
generate_lasernames(object,type,Vpct1);
object='trailer';
generate_lasernames(object,type,Vpct1);
object='wall';
generate_lasernames(object,type,Vpct2);
object='fence';
generate_lasernames(object,type,Vpct3);

type='radar';
object='car';
generate_lasernames(object,type,Vpct1);
object='trailer';
generate_lasernames(object,type,Vpct1);
object='wall';
generate_lasernames(object,type,Vpct2);
object='fence';
generate_lasernames(object,type,Vpct3);