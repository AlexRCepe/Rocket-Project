classdef Rocket

    properties
        rocketName;
        parts = RocketPart.empty; % Array of RocketPart objects
    end

    methods

        function obj = add_part(obj, part)
            %ADD_PART Add a RocketPart to the rocket
            %
            % Inputs:
            %   part - An instance of RocketPart or its subclass


            if isempty(obj.parts)

                obj.parts = part;

            else
                obj.parts(end + 1) = part;

                if isa(part, "FinSet")
                    % Dont do anythin until implemented

                else
                    last_part = obj.parts(end - 1);
                    part.update_position([0, 0, last_part.position(3) + last_part.length]);
                end

            end
        end


    end

end