classdef FinSet < SubRocketPart
    % Represents a set of fins attached to the rocket.

    properties

        num_fins;
        span;
        root_chord;
        tip_chord;
        sweep;
        thickness;
        mass_per_fin;
    end

    methods

        function obj = FinSet(name, num_fins, span, root_chord, tip_chord, sweep, thickness, mass_per_fin, parent_part)
            % Constructs an instance of the FinSet class.
            %
            % Inputs:
            %   name - Name of the fin set (string).
            %   num_fins - Number of fins in the set (integer).
            %   span - Span of a single fin (m).
            %   root_chord - Length of the fin chord at the root (m).
            %   tip_chord - Length of the fin chord at the tip (m).
            %   sweep - Sweep distance of the fin's leading edge (m).
            %   thickness - Thickness of the fins (m).
            %   mass_per_fin - Mass of a single fin (kg).
            %   parent_part - The RocketPart this component is attached to.

            total_mass = mass_per_fin * num_fins;
            
            obj@SubRocketPart(name, total_mass, parent_part);

            obj.num_fins = num_fins;
            obj.span = span;
            obj.root_chord = root_chord;
            obj.tip_chord = tip_chord;
            obj.sweep = sweep;
            obj.thickness = thickness;
            obj.mass_per_fin = mass_per_fin;

            obj.mass = obj.num_fins * mass_per_fin;

        end

        
        function cg = compute_cg(obj)
        % COMPUTE_CG Computes the longitudinal center of gravity of the fin set.
        %
        %   Output Arguments:
        %       cg - The longitudinal position of the fin set's center of
        %            gravity. [m]


            cg_long = (obj.sweep * (obj.root_chord + 2*obj.tip_chord)) / (3 * (obj.root_chord + obj.tip_chord)) + (obj.root_chord^2 + obj.root_chord*obj.tip_chord + obj.tip_chord^2) / (3 * (obj.root_chord + obj.tip_chord));

            cg = [0, 0, cg_long];

        end

        function d = get_fin_centroid(obj)
            % Computes the distance form the root chord to the fin centroid

            d = (obj.span / 3) * ( (obj.root_chord + 2*obj.tip_chord) / (obj.root_chord + obj.tip_chord) );
        end

        function I = compute_inertia(obj)
            % Computes the inertia tensor of the fin set about its own center of gravity
            %
            % Outputs:
            %   I - Inertia tensor as a 3x3 matrix

            d = obj.parent_part.diameter / 2 + obj.get_fin_centroid();
            cg = obj.compute_cg();
            z_cg = cg(3);
            
            Area_fin = (obj.span / 2) * (obj.root_chord + obj.tip_chord);

            I_chordwise = obj.mass_per_fin * ( (obj.span^2 / 18) * (obj.root_chord^2 + 4*obj.root_chord*obj.tip_chord + ...
                          obj.tip_chord^2) / (obj.root_chord + obj.tip_chord)^2 + (obj.thickness^2 / 12) );

            I_area_root = (obj.span / 12) * (obj.root_chord^3 + obj.tip_chord^3 + (obj.root_chord + obj.tip_chord) * ...
                          (obj.root_chord*obj.tip_chord + 3*obj.sweep^2) + 3*obj.sweep*(obj.root_chord^2 + obj.tip_chord^2));

            I_area_spanwise = I_area_root - Area_fin * z_cg^2;
            I_spanwise = obj.mass_per_fin * ( I_area_spanwise / Area_fin + (obj.thickness^2 / 12) );

            I_xx = obj.num_fins * (I_chordwise + obj.mass_per_fin * z_cg^2) + ...
                (obj.num_fins / 2) * (I_spanwise + obj.mass_per_fin * d^2);

            I_yy = I_xx; % By symmetry

            I_zz = obj.num_fins * ( I_spanwise + obj.mass_per_fin * d^2 );

            I = diag([I_xx, I_yy, I_zz]);

        end

    end
    
end