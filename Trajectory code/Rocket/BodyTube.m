classdef BodyTube < RocketPart

    properties

        diameter;
        length;
        thickness;

    end

    methods

        function obj = BodyTube(name, mass, diameter, length, thickness)
            %BODYTUBE Construct an instance of this class
            %
            % Inputs:
            %   name - Name of the cylinder (string)
            %   mass - Mass of the cylinder (kg)
            %   diameter - Diameter of the cylinder (m)
            %   length - Length of the cylinder (m)
            %   thickness - Thickness of the cylinder wall (m)
            %

            obj.name = name;
            obj.mass = mass;
            obj.diameter = diameter;
            obj.length = length;
            obj.thickness = thickness;
        end

        function cg = compute_cg(obj)
            %COMPUTE_CG Get the center of gravity of the cylinder
            %
            % Outputs:
            %   cg - Center of gravity [x, y, z] in meters

            cg = [0, 0, obj.length / 2];

        end

        function I = compute_inertia(obj)
            %COMPUTE_INERTIA Get the inertia tensor of the cylinder
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