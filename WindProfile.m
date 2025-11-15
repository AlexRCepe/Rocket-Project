classdef WindProfile
    % Represents a wind profile from a .csv file and allows interpolation.
    %
    % Properties:
    %   altitude - Altitude data (m)
    %   wind_speed - Wind speed data (m/s)
    %
    % Methods:
    %   WindProfile - Constructs a WindProfile object from a .csv file.
    %   get_wind_speed - Interpolates the wind speed at a given altitude.
    
    properties
        altitude
        wind_speed
    end
    
    methods
        function obj = WindProfile(filename)
            % Constructs a WindProfile object from a .csv file.
            %
            % Inputs:
            %   filename - The name of the .csv file.
            
            data = readmatrix(filename);
            obj.altitude = data(:, 1);
            obj.wind_speed = data(:, 2);
        end
        
        function speed = get_wind_speed(obj, h)
            % Interpolates the wind speed at a given altitude.
            %
            % Inputs:
            %   h - The altitude (m).
            %
            % Outputs:
            %   speed - The interpolated wind speed (m/s).
            
            speed = interp1(obj.altitude, obj.wind_speed, h, 'linear', 'extrap');
        end
    end
end
