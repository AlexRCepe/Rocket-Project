classdef Mass < RocketPart
    % Mass represents a simple mass component of a rocket.
    % This class is used for components that can be modeled as point masses.

    properties

        length = 0;
        diameter = 0;

    end

    methods

        function obj = Mass(name, mass, length, diameter)
            %MASS Construct an instance of this class
            %
            % Inputs:
            %   name - Name of the mass (string)
            %   mass - Mass of the component (kg)
            %   length - Length of the mass (m)
            %   diameter - Diameter of the mass (m)

            obj.name = name;
            obj.mass = mass;
            obj.length = length;
            obj.diameter = diameter;

        end

        function cg = compute_cg(obj)
            %COMPUTE_CG Get the center of gravity of the mass
            %
            % Outputs:
            %   cg - Center of gravity [x, y, z] in meters

            cg = [0, 0, obj.length / 2]; % Assuming uniform density along length

        end

        function I = compute_inertia(obj)
            %COMPUTE_INERTIA Get the inertia tensor of the mass
            %
            % Outputs:
            %   I - Inertia tensor as a 3x3 matrix

            r = obj.diameter / 2; % Approximate radius
            l = obj.length;
            m = obj.mass;

            Ixx = (1/4) * m * r^2 + (1/12) * m * l^2;
            Iyy = Ixx;
            Izz = 0.5 * m * r^2;

            I = diag([Ixx, Iyy, Izz]);
        end
    end
end