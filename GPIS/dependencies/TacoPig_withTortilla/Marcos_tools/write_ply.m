% pty generator ... 

function write_ply (filename,M)
% Generates a *.ply file based on a matrix M
% example of usage

%write_pty('test.ply',t)

%note for laser files i will be the upper confidence uncertainty level and
% confidece will be the lower band of confidence ... :S
% Calling the function example: 

% write_pty(filename,t')

% t= [posx(npos) posy(npos) posz(npos)];
% tic
% formatSpec = '%d %d %d\n';
% filename= ('../datasets/surfaces/laser_surface_tcalib_compresor_sqexp_res150.xyz');
% fileID = fopen(filename,'w');
% fprintf(fileID,formatSpec,t');
% fclose(fileID); % around 8 times faster than dlmrwrite...
% display('file ... done');
% toc

fileID = fopen(filename,'w');
%header
vs=size(M,2);
fprintf(fileID, '%s\n%s\n%s\n%s\t%d\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n', 'ply','format ascii 1.0','comment zipper output',...
'element vertex', vs, 'property float x', 'property float y', 'property float z',...
'property float intensity', 'element face 0', 'property list uchar int vertex_indices', ...
'end_header');
% data 
formatSpec = '%f %f %f %f\n';
fprintf(fileID,formatSpec,M); % Writing intensities ...
% 
% 
% 
% 
% header="
% 
% ply 
% format ascii 1.0
% comment zipper output
% element vertex 453
% property float x
% property float y
% property float z
% %property float confidence
% property float intensity
% element face 0
% property list uchar int vertex_indices
% end_header";
