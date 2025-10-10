classdef Parachute < SubRocketPart
    % Represents a parachute, including its deployment characteristics.

    properties
        diameter;
        drag_coefficient;
        reference_area;
        deployment_altitude;
        length;
    end

    methods 
        function obj = Parachute(name, diameter, drag_coefficient, deployment_altitude, length, mass, parent_part)
            % Constructs an instance of the Parachute class.
            %
            % Inputs:
            %   name - Name of the parachute (string).
            %   diameter - Deployed diameter of the parachute (m).
            %   drag_coefficient - Drag coefficient when deployed.
            %   deployment_altitude - Altitude at which the parachute deploys (m).
            %   length - Packed length of the parachute component (m).
            %   mass - Mass of the parachute (kg).
            %   parent_part - The RocketPart this component is attached to.

            obj@SubRocketPart(name, mass, parent_part);
            
            obj.mass = mass;
            obj.name = name;
            obj.diameter = diameter;
            obj.drag_coefficient = drag_coefficient;
            obj.deployment_altitude = deployment_altitude;
            obj.length = length;

            obj.reference_area = pi * (diameter / 2)^2; % Reference area based on diameter

        end

        function cg = compute_cg(obj)
            % Computes the center of gravity of the packed parachute.
            %
            % Outputs:
            %   cg - The center of gravity [x, y, z] in meters, relative to the part's origin.

            cg = [0, 0, obj.length / 2]; % Center of gravity is at half the length

        end

        function I = compute_inertia(obj)
            % Computes the inertia tensor of the packed parachute, modeled as a cylinder.
            %
            % Outputs:
            %   I - The inertia tensor as a 3x3 matrix.

            r = obj.diameter / 2; % Radius

            Izz = 0.5 * obj.mass * r^2; % Moment of inertia about the z-axis
            Ixx = (1/4) * obj.mass * r^2 + (1/12) * obj.mass * obj.length^2; 
            Iyy = Ixx; 

            I = diag([Ixx, Iyy, Izz]);

        end

    end