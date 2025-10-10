classdef Parachute < RocketPart

    properties
        diameter;
        drag_coefficient;
        reference_area;
        deployment_altitude;
        length;
    end

    methods 
        function obj = Parachute(name, diameter, drag_coefficient, deployment_altitude, length, mass)
            %PARACHUTE Construct an instance of this class
            %
            % Inputs:
            %   name - Name of the parachute (string)
            %   diameter - Diameter of the parachute (m)
            %   drag_coefficient - Drag coefficient of the parachute (dimensionless)
            %   deployment_altitude - Altitude at which the parachute deploys (m)
            %   length - Length of the parachute (m)
            %   mass - Mass of the parachute (kg)

            
            obj.mass = mass;
            obj.name = name;
            obj.diameter = diameter;
            obj.drag_coefficient = drag_coefficient;
            obj.deployment_altitude = deployment_altitude;
            obj.length = length;

            obj.reference_area = pi * (diameter / 2)^2; % Reference area based on diameter

        end

        function cg = compute_cg(obj)
            % COMPUTE_CG Computes the center of gravity of the parachute.
            %
            %   Output Arguments:
            %       cg - The position of the parachute's center of
            %            gravity. [m]

            cg = [0, 0, obj.length / 2]; % Center of gravity is at half the length

        end

        function I = compute_inertia(obj)
            % COMPUTE_INERTIA Computes the inertia tensor of the parachute.
            %
            %   Output Arguments:
            %       I - The inertia tensor as a 3x3 matrix.

            r = obj.diameter / 2; % Radius

            Izz = 0.5 * obj.mass * r^2; % Moment of inertia about the z-axis
            Ixx = (1/4) * obj.mass * r^2 + (1/12) * obj.mass * obj.length^2; 
            Iyy = Ixx; 

            I = diag([Ixx, Iyy, Izz]);

        end

    end