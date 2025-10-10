classdef RocketPart
    % Represents a generic component of a rocket.
    % This is an abstract base class that defines the common interface for all
    % rocket parts, including properties like name, mass, and position, and
    % methods for calculating center of gravity and inertia.

    properties
        name
        mass = 0.0; % Mass is overriden when instatiating the object
        position = [0, 0, 0]; % Position of the part in 3D space [x, y, z]   
    end

    methods

        function obj = RocketPart(name, mass)

            obj.name = name;
            obj.mass = mass;

        end

        function cg = compute_cg(obj)
            % Computes the center of gravity of the rocket part.
            %
            % Outputs:
            %   cg - Center of gravity [x, y, z] in meters.

            cg = [0, 0, 0]; % Placeholder for center of gravity calculation
        end

        function I = compute_inertia(obj)
            % Computes the inertia tensor of the rocket part.
            %
            % Outputs:
            %   I - Inertia tensor as a 3x3 matrix.

            I = eye(3); % Placeholder for inertia tensor calculation
        end

        function obj = update_position(obj, new_position)
            % Updates the position of the rocket part.
            %
            % Inputs:
            %   new_position - New position [x, y, z] in meters.

            obj.position = new_position;
        end

    end

end