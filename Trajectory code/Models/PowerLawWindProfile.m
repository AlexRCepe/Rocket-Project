classdef PowerLawWindProfile < WindModel
    % Implements a power law wind model.
    % This model describes how wind speed changes with altitude according to a power law relationship.
    % Reference: Davenport, A. G. (1965). The relationship of wind structure to wind loading

    properties
        ref_speed       % Wind speed at the reference height (m/s)
        ref_height      % Reference height (m)
        alpha           % Power law exponent (dimensionless)
        direction_deg   % Wind direction in degrees (0=from North, 90=from East)
    end

    methods
        function obj = PowerLawWindProfile(ref_speed, ref_height, alpha, direction_deg)
            % Constructs an instance of the PowerLawWindProfile class.
            %
            % Inputs:
            %   ref_speed - Wind speed at the reference height (m/s). Default: 5.0.
            %   ref_height - Reference height (m). Default: 500.0.
            %   alpha - Power law exponent. Default: 1/7.
            %   direction_deg - Wind direction in degrees. Default: 270 (from West).
            
            arguments
                ref_speed = 5.0;
                ref_height = 500.0;
                alpha = 1/7;
                direction_deg = 270;
            end

            obj.ref_speed = ref_speed;
            obj.ref_height = ref_height;
            obj.alpha = alpha;
            obj.direction_deg = direction_deg;

        end

        function [vx, vy] = wind_at_h(obj, height)
            % Calculates the wind vector components at a given height.
            %
            % Inputs:
            %   height - Altitude above ground level (m).
            %
            % Outputs:
            %   vx - The x-component of the wind velocity (m/s).
            %   vy - The y-component of the wind velocity (m/s).

            if height <= 0
                vx = 0;
                vy = 0;
                return;
            end

            speed_at_h = obj.ref_speed * (height / obj.ref_height)^obj.alpha;

            % 0 deg (North) -> 270 deg; 90 deg (East) -> 180 deg.
            direction_rad = deg2rad(270 - obj.direction_deg);

            vx = speed_at_h * cos(direction_rad);
            vy = speed_at_h * sin(direction_rad);
        end
    end
end