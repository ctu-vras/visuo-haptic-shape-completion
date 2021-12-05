function write_file(basename,type,t,tt,ttt)
%laser_surface_compresor_thin056_exp.ply
%rawout_filename=('/home/marcos/Desktop/gp_matlab_original/datasets/postprocessed/laser_rawout_exp_compresor_thin056.csv');
try 
    %nargin
    switch nargin
        case 2
            t=[];
            tt=[];
            ttt=[];
        case 3
            tt=[];
            ttt=[];
        case 4
            ttt=[];
        case 5
        otherwise
    end
              
  
    if strcmp(type,'laser') 
        fprintf('Processing laser %s ...\n', basename);
        path='/home/marcos/Desktop/gp_matlab_original/datasets/surfaces/';
        surface=sprintf('%slaser_surface_%s.ply',path,basename);
        original=sprintf('%slaser_original_%s.xyz',path,basename);
        raw=sprintf('/home/marcos/Desktop/gp_matlab_original/datasets/postprocessed/laser_rawout_%s.csv',basename);
            
        
        
        if (~isempty(t))
            fprintf('%s\n',surface);
            write_ply(surface,t')
        else
            display('Laser Surface was not Written');
        end
        
        if (~isempty(tt))
             fprintf('%s\n',original);
             formatSpec = '%d %d %d\n';
             fileID = fopen(original,'w');
             fprintf(fileID,formatSpec,tt);
             fclose(fileID); 
        else
            display('Laser Original was not Written');
        end
        
        
        if (~isempty(ttt))
            
            fprintf('%s\n',raw);
             fileID = fopen(raw,'w');
             formatSpec = '%f %f %f %f %f\n';
             fprintf(fileID,formatSpec,ttt);
        else
            display('Laser Raw was not Written');
        end


    elseif strcmp(type,'radar')
        fprintf('Processing radar %s ...\n', basename)
        path='/home/marcos/Desktop/gp_matlab_original/datasets/surfaces/';
        surface=sprintf('%sradar_surface_%s.ply', path,basename);
        original=sprintf('%sradar_original_%s.ply', path,basename);
        raw=sprintf('/home/marcos/Desktop/gp_matlab_original/datasets/postprocessed/radar_rawout_%s.csv',basename);
        
        if (~isempty(t))
            fprintf('%s\n',surface);
            write_ply(surface,t')
        else
            display('Radar Surface was not Written');
        end
        
        if (~isempty(tt))
            fprintf('%s\n',original);
            write_ply(original,tt)
        else
            display('Radar Original was not Written');
        end
        
        if (~isempty(ttt))
            fprintf('%s\n',raw);
            formatSpec = '%d %d %d\n';
            fileID = fopen(raw,'w');
            fprintf(fileID,formatSpec,ttt);
            fclose(fileID); 
        else
            display('Radar Raw was not Written');
        end
        

            
    elseif strcmp(type,'fusion')
        fprintf('Processing fusion %s ...\n', basename)
        path='/home/marcos/Desktop/gp_matlab_original/datasets/surfaces/';
        surface=sprintf('%sfusion_surface_%s.ply', path,basename);
        original=sprintf('%sfusion_original_%s.ply', path,basename);
        raw=sprintf('/home/marcos/Desktop/gp_matlab_original/datasets/postprocessed/fusion_rawout_%s.csv',basename);

        fprintf('%s\n',surface);
        fprintf('%s\n',original);
        fprintf('%s\n',raw);

    else
        display('Warning, type not valid. Nothing was writted...')
          
    end
catch err
    rethrow(err)
end


    
    
    

