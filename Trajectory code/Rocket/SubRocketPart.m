classdef SubRocketPart < RocketPart
    % Represents a component that is attached to a parent part.
    % This class extends RocketPart to include a reference to a parent 
    % and provides a method to update its position relative to that parent.

    properties
        parent_part % The part this component is attached to
    end

    methods
        function obj = SubRocketPart(name, mass, parent_part)
            % Constructs an instance of the SubRocketPart class.
            %
            % Inputs:
            %   name - Name of the sub-part (string).
            %   mass - Mass of the sub-part (kg).
            %   parent_part - The RocketPart this component is attached to.

            obj@RocketPart(name, mass);
            obj.parent_part = parent_part;
        end

        function obj = update_position(obj, relative_position)
            % Updates the part's absolute position based on its parent's position.
            %
            % Inputs:
            %   relative_position - The [x, y, z] position relative to the parent part's origin (m).
            
            if ~isempty(obj.parent_part) && isprop(obj.parent_part, 'position')
                obj.position = obj.parent_part.position + relative_position;
            else
                obj.position = relative_position;
            end
        end
    end
end