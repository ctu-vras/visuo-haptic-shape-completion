% pty generator ... 

function write_pty (filename,M)
% Generates a ply 
% confidece will be the lower band of confidence ... :S
% Calling the function example: 
% t= [posx(npos) posy(npos) posz(npos) abs(f(npos))];
% write_pty(filename,t')
% Where filemane may be e.g. 'file.ply'
% posx,posy and posz are the x,y,z positions of the mesh
% abs(f(npos)) are the uncertainty related to those points ... 

fileID = fopen(filename,'w');
vs=size(M,2); matrix size
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
