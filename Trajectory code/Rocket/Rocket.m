classdef Rocket
    % Represents the entire rocket assembly, composed of multiple RocketPart objects.
    % This class manages the collection of rocket parts and calculates
    % the rocket's overall mass, center of gravity (CG), and inertia at any given time.

    properties
        rocketName;
        parts = {}; % Use a cell array to store parts of different classes
    end

    methods

        function obj = add_part(obj, part)
            % Adds a RocketPart to the rocket.
            %
            % Inputs:
            %   part - An instance of a class that inherits from RocketPart.

            % Add the new part to the cell array
            obj.parts{end + 1} = part;
            
            if length(obj.parts) > 1 && ~isa(part, "SubRocketPart")

                last_part = obj.parts{end - 1};
                
                part.position = [0, 0, last_part.position(3) + last_part.length];
                obj.parts{end} = part; 
                
            end
        end

        function total_mass = get_total_mass(obj, time)
            % Calculates the total mass of the rocket at a given time.
            %
            % Inputs:
            %   time - Time in seconds (s).
            %
            % Outputs:
            %   total_mass - Total mass of the rocket (kg).

            total_mass = 0;

            for i = 1:length(obj.parts)
                part = obj.parts{i}; % Access part from cell array
                if isa(part, "Motor")
                    total_mass = total_mass + part.get_mass_at_time(time);
                else
                    total_mass = total_mass + part.mass;
                end

            end
        end

        function cg = get_cg(obj, time)
            % Calculates the center of gravity (CG) of the rocket at a given time.
            %
            % Inputs:
            %   time - Time in seconds (s).
            %
            % Outputs:
            %   cg - Center of gravity position (m) from the rocket's tip.

            total_mass = obj.get_total_mass(time);

            if total_mass == 0
                cg = 0;
                return;
            end

            moment_sum = 0;
            for i = 1:length(obj.parts)
                part = obj.parts{i}; % Access part from cell array
                if isa(part, "Motor")
                    part_mass = part.get_mass_at_time(time);
                    part_cg_local = part.get_cg_at_time(time);
                    part_cg_wrt_tip = part.position(3) + part_cg_local;
                else
                    part_mass = part.mass;
                    part_cg_local = part.compute_cg();
                    part_cg_wrt_tip = part.position(3) + part_cg_local(3); % Assuming uniform density
                end

                moment_sum = moment_sum + part_mass * part_cg_wrt_tip;

            end

            cg = moment_sum / total_mass;

        end

        function I = get_inertia(obj, time)

            % Calculates the moment of inertia of the rocket at a given time.
            %
            % Inputs:
            %   time - Time in seconds (s).
            %
            % Outputs:
            %   I - Moment of inertia tensor (kg*m^2) about the rocket's center of gravity.

            cg = obj.get_cg(time);
            I = zeros(3,3); % Initialize as a 3x3 zero matrix

            for i = 1:length(obj.parts)
                part = obj.parts{i}; % Access part from cell array
                if isa(part, "Motor")
                    
                    part_mass = part.get_mass_at_time(time);
                    part_cg_local = part.get_cg_at_time(time);
                    part_cg = part.position(3) + part_cg_local;
                    part_I = part.get_inertia_at_time(time);

                else

                    part_mass = part.mass;
                    part_cg_local = part.compute_cg();
                    part_cg = part.position(3) + part_cg_local(3);
                    part_I = part.compute_inertia();
                    
                end
                
                % Apply parallel axis theorem
                d_vec = [0, 0, part_cg - cg];
                d_sq = dot(d_vec, d_vec);
                I = I + part_I + part_mass * (d_sq * eye(3) - d_vec' * d_vec);


            end
        end
    end
end