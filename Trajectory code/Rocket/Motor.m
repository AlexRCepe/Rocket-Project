classdef Motor < RocketPart

    % TODO: evolucion del cg y de la inercia con el consumo del propelente<

    properties (visible = 'private')

        p_sl = 101325;

    end

    properties

        nozzle_diameter;
        length;
        total_mass;
        propellant_mass;
        thrust_curve; % Nx2 array: [time (s), thrust (N)]
    end

    methods
        
        function obj = Motor(name, total_mass, propellant_mass, nozzle_diameter, length, thrust_curve)
            %MOTOR Construct an instance of this class
            %
            % Inputs:
            %   name - Name of the motor (string)
            %   total_mass - Total mass of the motor (kg)
            %   propellant_mass - Mass of the propellant (kg)
            %   nozzle_diameter - Diameter of the nozzle exit (m)
            %   length - Length of the motor (m)
            %   thrust_curve - Nx2 array: [time (s), thrust (N)]

            obj.name = name;
            obj.total_mass = total_mass;
            obj.propellant_mass = propellant_mass;
            obj.nozzle_diameter = nozzle_diameter;
            obj.length = length;
            obj.thrust_curve = thrust_curve;

            obj.exit_area = pi * (nozzle_diameter / 2)^2;

        end

        function cg = compute_cg(obj)
            %COMPUTE_CG Get the center of gravity of the motor
            %
            % Outputs:
            %   cg - Center of gravity [x, y, z] in meters

            cg = [0, 0, obj.length / 2]; % Assuming uniform density along length

        end

        function I = compute_inertia(obj)
            %COMPUTE_INERTIA Get the inertia tensor of the motor
            %
            % Outputs:
            %   I - Inertia tensor as a 3x3 matrix

            r = obj.nozzle_diameter / 2; % Approximate radius
            l = obj.length;
            m = obj.total_mass;

            Ixx = (1/4) * m * r^2 + (1/12) * m * l^2;
            Iyy = Ixx;
            Izz = 0.5 * m * r^2;

            I = diag([Ixx, Iyy, Izz]);
        end

        function thrust = get_thrust_at_time(obj, time, p_atm)
            %GET_THRUST_AT_TIME Get the thrust at a specific time
            %
            % Inputs:
            %   time - Time in seconds
            %   p_atm - Atmospheric pressure in Pascals (not used in this simple model)
            %
            % Outputs:
            %   thrust - Thrust in Newtons

            if time < 0 || time > obj.thrust_curve(end, 1)
                thrust = 0;
                return;
            end
            
            thrust_sl = interp1(obj.thrust_curve(:, 1), obj.thrust_curve(:, 2), time, 'linear', 0);

            thrust = thrust_sl + obj.exit_area * (obj.p_sl - p_atm);

        end

    end
end