classdef NoseCone < RocketPart
    % Represents the nose cone of the rocket.
    % Calculates CG and inertia based on its shape.
    % TODO: Change once the shape is known for sure.

    properties
        
        shape; % 'conical', 'ogive', 'parabolic', 'elliptical'
        length; % Length of the nose cone [m]
        base_diameter; % Base diameter of the nose cone [m]
        mass; % Mass of the nose
        parent_cylinder; % Reference to the parent rocket cylinder

    end
    
    methods
        function obj = NoseCone(name, shape, length, base_diameter, mass, parent_cylinder)
            % Constructs an instance of the NoseCone class.
            %
            % Inputs:
            %   name - Name of the nose cone (string).
            %   shape - Shape of the nose cone ('conical', 'ogive', etc.).
            %   length - Length of the nose cone (m).
            %   base_diameter - Diameter of the nose cone base (m).
            %   mass - Mass of the nose cone (kg).
            %   parent_cylinder - Reference to the parent body tube.

            obj.mass = mass;
            obj.name = name;

            obj.shape = shape;
            obj.length = length;
            obj.base_diameter = base_diameter;
            obj.mass = mass;
            obj.parent_cylinder = parent_cylinder;

        end
        
        function cg = compute_cg(obj)
            % Computes the center of gravity of the nose cone.
            % CG is measured from the tip (z=0) of the nose cone.
            %
            % Outputs:
            %   cg - Center of gravity [x, y, z] in meters, relative to the part's origin.
            
            L = obj.length;

            switch lower(obj.shape)
                case 'conical'

                    cg_long = (2/3) * L;

                case 'ogive'

                    cg_long = 0.46 * L;

                case 'parabolic'

                    cg_long = (2/3) * L;

                case 'elliptical'

                    cg_long = (5/8) * L;

                otherwise

                    error('Unsupported nose cone shape: %s', obj.shape);
            end
            
            cg = [0, 0, cg_long];
        end
        
        function I = compute_inertia(obj)
            % Computes the inertia tensor of the nose cone about its own center of gravity.
            %
            % Outputs:
            %   I - Inertia tensor as a 3x3 matrix.
            
            r = obj.base_diameter / 2;
            L = obj.length;
            m = obj.mass;
            
            switch lower(obj.shape)
                case 'conical'

                    Izz = (3/10) * m * r^2; 
                    Ixx = m * ( (3/20)*r^2 + (3/80)*L^2 ); 

                case 'ogive'

                    % Uses conical as approximation
                    Izz = (3/10) * m * r^2;
                    Ixx = m * ( (3/20)*r^2 + (3/80)*L^2 );

                case 'parabolic'

                    Izz = (1/3) * m * r^2;
                    Ixx = m * ( (1/4)*r^2 + (1/9)*L^2 );

                case 'elliptical'

                    Izz = (2/5) * m * r^2;
                    Ixx = m * ( (1/5)*r^2 + (5/32)*L^2 );

                otherwise

                    error('Unsupported nose cone shape: %s', obj.shape);
            end
            
            Iyy = Ixx;
            
            I = diag([Ixx, Iyy, Izz]);
        end
    end
end