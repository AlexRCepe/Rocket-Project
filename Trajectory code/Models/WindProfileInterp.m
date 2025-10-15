classdef WindProfileInterp < WindModel

    properties
        heights   
        vx_vec
        vy_vec
    end

    methods

        function obj = WindProfileInterp(file_path)
            % Constructs a WindProfileInterp object from a data file.
            % The file is expected to have three columns: height, vx, and vy,
            % and may contain a header row.
            %
            % Inputs:
            %   file_path - String path to the wind profile data file.
            
            data = readmatrix(file_path);
            
            obj.heights = data(:, 1);
            obj.vx_vec = data(:, 2);
            obj.vy_vec = data(:, 3);
        end

        
        function [vx, vy] = wind_at_h(obj, height)
            % Interpolates wind velocity components at a specific height using linear
            % interpolation and extrapolation on the stored wind profile data.
            %
            %   Inputs:
            %       obj     - The object instance containing the wind profile data
            %                 (properties: heights, vx_vec, vy_vec).
            %       height  - Scalar or vector of altitudes [m] at which to
            %                 interpolate the wind velocity.
            %
            %   Outputs:
            %       vx      - Interpolated x-component(s) of wind velocity [m/s].
            %       vy      - Interpolated y-component(s) of wind velocity [m/s].
            %

            vx = interp1(obj.heights, obj.vx_vec, height, 'linear', 'extrap');
            vy = interp1(obj.heights, obj.vy_vec, height, 'linear', 'extrap');
        end
    end

end