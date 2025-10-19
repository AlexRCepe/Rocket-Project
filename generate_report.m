% filepath: generate_report.m
% This script generates a PDF report containing the source code of the
% rocket trajectory and motor burn simulation.

clc
clear
close all

% --- Configuration ---
report_filename = 'Rocket_Project_Code_Report.pdf';
temp_script_name = 'temp_report_generator.m';
files_to_include = {
    'trajectory.m', 'Trajectory Simulation Code';
    'get_curves.m', 'Thrust and Mass Flow Curve Generation';
    'project_grain.m', 'Grain Burn Simulation';
    'project_geosolver.m', 'Grain Geometric Solver'
};

% --- Script Generation ---
try
    % Create a temporary script to be published
    fid = fopen(temp_script_name, 'w');
    if fid == -1
        error('Cannot create temporary file for the report.');
    end

    fprintf(fid, '%%%% Rocket Project Code Report\n');
    fprintf(fid, '%% This document contains the source code for the main components of the rocket simulation project.\n\n');

    for i = 1:size(files_to_include, 1)
        filename = files_to_include{i, 1};
        description = files_to_include{i, 2};
        
        fprintf(fid, '%%%% %s (%s)\n', description, filename);
        fprintf(fid, '%% The following code is from the file |%s|.\n\n', filename);
        fprintf(fid, 'type(''%s'');\n\n', filename);
    end
    
    fclose(fid);

    % --- PDF Publishing ---
    fprintf('Generating PDF report...\n');
    
    % Set publish options
    publish_options.format = 'pdf';
    publish_options.outputDir = '.';
    publish_options.showCode = false; % The code is displayed by `type`

    % Publish the temporary script
    published_file = publish(temp_script_name, publish_options);
    
    % Rename the output file
    movefile(published_file, report_filename, 'f');
    
    fprintf('Report successfully generated: %s\n', report_filename);

catch ME
    fprintf('An error occurred while generating the report:\n');
    fprintf('%s\n', ME.message);
end

% --- Cleanup ---
% Delete the temporary script and its generated folder
if exist(temp_script_name, 'file')
    delete(temp_script_name);
end
if exist('html', 'dir')
    rmdir('html', 's');
end