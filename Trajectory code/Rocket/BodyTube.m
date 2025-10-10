classdef BodyTube < RocketPart
    % Represents a cylindrical section of the rocket, such as the main airframe.

    properties

        diameter;
        length;
        thickness;

    end

    methods

        function obj = BodyTube(name, mass, diameter, length, thickness)
            % Constructs an instance of the BodyTube class.
            %
            % Inputs:
            %   name - Name of the body tube (string).
            %   mass - Mass of the body tube (kg).
            %   diameter - Outer diameter of the body tube (m).
            %   length - Length of the body tube (m).
            %   thickness - Wall thickness of the body tube (m).
            %

            obj@RocketPart(name, mass);

            obj.diameter = diameter;
            obj.length = length;
            obj.thickness = thickness;
        end

        function cg = compute_cg(obj)
            % Computes the center of gravity of the body tube.
            %
            % Outputs:
            %   cg - Center of gravity [x, y, z] in meters, relative to the part's origin.

            cg = [0, 0, obj.length / 2];

        end

        function I = compute_inertia(obj)
            % Computes the inertia tensor of the body tube.
            %
            % Outputs:
            %   I - Inertia tensor as a 3x3 matrix.
            r_out = obj.diameter / 2;
            r_in = r_out - obj.thickness; % Inner radius

            l = obj.length;
            m = obj.mass;

            Ixx = (1/4) * m * (r_in^2 + r_out^2) + (1/12) * m * l^2;
            Iyy = Ixx;
            Izz = 0.5 * m * (r_in^2 + r_out^2);

            I = diag([Ixx, Iyy, Izz]);
        end

    end

end