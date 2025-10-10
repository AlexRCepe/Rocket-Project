classdef RocketPart
    % RocketPart represents a component of a rocket.
    % Main properties and methods should be defined here.

    properties
        name
        mass = 0.0; % Mass is overriden when instatiating the object
        position = [0, 0, 0]; % Position of the part in 3D space [x, y, z]   
    end

    methods

        function cg = compute_cg(obj)
            %COMPUTE_CG Get the center of gravity of the rocket part
            %
            % Outputs:
            %   cg - Center of gravity [x, y, z] in meters

            cg = [0, 0, 0]; % Placeholder for center of gravity calculation
        end

        function I = compute_inertia(obj)
            %COMPUTE_INERTIA Get the inertia tensor of the rocket part
            %
            % Outputs:
            %   I - Inertia tensor as a 3x3 matrix

            I = eye(3); % Placeholder for inertia tensor calculation
        end

        function obj = update_positon(obj, new_position)
            %UPDATE_POSITION Update the position of the rocket part
            %
            % Inputs:
            %   new_position - New position [x, y, z] in meters

            obj.position = new_position;
        end

    end

end