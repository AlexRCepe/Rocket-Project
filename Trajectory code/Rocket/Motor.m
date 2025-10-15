classdef Motor < SubRocketPart
    % Represents the rocket motor, including propellant mass and thrust characteristics.
    % TODO: Implement the evolution of CG and inertia as propellant is consumed.

    properties (Access = private)

        exit_area;

    end

    properties

        p_sl = 101325; % Sea level atmospheric pressure (Pa)
        nozzle_diameter;
        length;
        total_mass;
        propellant_mass;
        thrust_curve; % Nx2 array: [time (s), thrust (N)]
    end

    methods
        
        function obj = Motor(name, total_mass, propellant_mass, nozzle_diameter, length, thrust_curve, parent_part)
            % Constructs an instance of the Motor class.
            %
            % Inputs:
            %   name - Name of the motor (string).
            %   total_mass - Initial total mass of the motor (casing + propellant) (kg).
            %   propellant_mass - Mass of the propellant (kg).
            %   nozzle_diameter - Diameter of the nozzle exit (m).
            %   length - Length of the motor (m).
            %   thrust_curve - Nx2 array with time (s) and thrust at sea level (N).
            %   parent_part - The RocketPart this component is attached to.

            obj@SubRocketPart(name, total_mass, parent_part);
            
            obj.total_mass = total_mass;
            obj.propellant_mass = propellant_mass;
            obj.nozzle_diameter = nozzle_diameter;
            obj.length = length;
            obj.thrust_curve = thrust_curve;

            obj.exit_area = pi * (obj.nozzle_diameter / 2)^2;

        end

        function thrust = get_thrust_at_time(obj, time, p_atm)
            % Gets the thrust at a specific time, adjusted for atmospheric pressure.
            %
            % Inputs:
            %   time - Time since ignition (s).
            %   p_atm - Ambient atmospheric pressure (Pa).
            %
            % Outputs:
            %   thrust - Thrust in Newtons (N).

            if time < 0 || time > obj.thrust_curve(end, 1)
                thrust = 0;
                return;
            end

            if nargin < 3 %! CHANGE ONCE THE ATMOSPHERE MODEL IS IMPLEMENTED
                p_atm = 101325; % Default to sea level pressure if not provided
            end
            
            thrust_sl = interp1(obj.thrust_curve(:, 1), obj.thrust_curve(:, 2), time, 'linear', 0);

            thrust = thrust_sl + obj.exit_area * (obj.p_sl - p_atm);

        end

        function mass = get_mass_at_time(obj, time)

            mass = 0;
            % TODO: Implement class equations assuming that the propellant is BATES

        end

        function cg = get_cg_at_time(obj, time)

            cg = 0;
            % TODO: Implement class equations assuming that the propellant is BATES

        end

        function I = get_inertia_at_time(obj, time)

            I = 0;
            % TODO: Implement class equations assuming that the propellant is BATES

        end

    end
end