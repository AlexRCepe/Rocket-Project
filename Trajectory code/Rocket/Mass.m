classdef Mass < SubRocketPart
    % Represents a simple mass component that can be modeled as a point mass or simple cylinder.

    properties

        length = 0;
        diameter = 0;
        
    
    end

    methods

        function obj = Mass(name, mass, length, diameter, parent_part)
            % Constructs an instance of the Mass class.
            %
            % Inputs:
            %   name - Name of the mass component (string).
            %   mass - Mass of the component (kg).
            %   length - Length of the mass (m).
            %   diameter - Diameter of the mass (m).
            %   parent_part - The RocketPart this component is attached to.

            obj@SubRocketPart(name, mass, parent_part);
            obj.mass = mass;
            obj.length = length;
            obj.diameter = diameter;

        end

        function cg = compute_cg(obj)
            % Computes the center of gravity of the mass.
            %
            % Outputs:
            %   cg - Center of gravity [x, y, z] in meters, relative to the part's origin.

            cg = [0, 0, obj.length / 2]; % Assuming uniform density along length

        end

        function I = compute_inertia(obj)
            % Computes the inertia tensor of the mass, modeled as a cylinder.
            %
            % Outputs:
            %   I - Inertia tensor as a 3x3 matrix.

            r = obj.diameter / 2; % Approximate radius
            Ixx = (1/4) * obj.mass * r^2 + (1/12) * obj.mass * obj.length^2;
            Iyy = Ixx;
            Izz = 0.5 * obj.mass * r^2;

            I = diag([Ixx, Iyy, Izz]);
        end
    end
end