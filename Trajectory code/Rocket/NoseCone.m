classdef NoseCone < RocketPart
    % TODO: chhange once the shape is known for sure

    properties
        
        shape; % 'conical', 'ogive', 'parabolic', 'elliptical'
        length; % Length of the nose cone [m]
        base_diameter; % Base diameter of the nose cone [m]
        mass; % Mass of the nose
        parent_cylinder; % Reference to the parent rocket cylinder

    end
    
    methods
        function obj = NoseCone(name, shape, length, base_diameter, mass, parent_cylinder)

            obj.mass = mass;
            obj.name = name;

            obj.shape = shape;
            obj.length = length;
            obj.base_diameter = base_diameter;
            obj.mass = mass;
            obj.parent_cylinder = parent_cylinder;

        end
        
        function cg = compute_cg(obj)
            % COMPUTE_CG Computes the longitudinal center of gravity of the nose cone.
            % CG is measured from the tip (z=0) of the nose cone.
            
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
            % COMPUTE_INERTIA Computes the inertia tensor of the nose cone
            % about its own center of gravity.
            
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