clc
clear

import mlreportgen.report.*
import mlreportgen.dom.*

rpt = Report('trajectory_report','pdf');

% Get the root folder of the project by going up one level from the script's directory
scriptPath = mfilename('fullpath');
scriptFolder = fileparts(scriptPath);
projectRoot = fileparts(scriptFolder);

% Find all .m files within the project directory
mFiles = dir(fullfile(projectRoot,'**','*.m'));

for k = 1:numel(mFiles)
    % Exclude the report generator script itself and empty files from the report
    if ~strcmp(mFiles(k).name, 'trajectory_report.m') && mFiles(k).bytes > 0
        filePath = fullfile(mFiles(k).folder, mFiles(k).name);
        % Create a relative path for the chapter title
        relative_path = strrep(filePath, [projectRoot, filesep], '');
        chapter = Chapter(relative_path);
        % Use MATLABCode reporter to add the file content
        add(chapter, mlreportgen.report.MATLABCode(filePath));
        add(rpt, chapter);
    end
end

close(rpt);
rptview(rpt);