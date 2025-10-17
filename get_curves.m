function [thrust_curve, mass_flow_curve] = get_curves(r, Me, epsilon, gamma, options)
% Calculates the thrust curve for a solid rocket motor with a star grain.
%
% Inputs:
%   r                - Initial radius parameter for the star points [in]
%   Me               - Mach number at the exit of the nozzle
%   epsilon          - Nozzle expansion ratio (Ae/At)
%   gamma            - Ratio of specific heats
%   options.Dt       - Throat diameter [in]
%   options.c_star   - Characteristic velocity [ft/s]
%
% Outputs:
%   thrust_curve     - A [Nx2] matrix of [time, thrust] data

    arguments
        r
        Me
        epsilon
        gamma
        options.Dt
        options.c_star
        options.g = 32.2;
        options.displayCurves = true;
    end

    At = pi * options.Dt^2 / 4; % [in^2]

    [t, Pc, ~] = project_grain(r, "Dt", options.Dt, "c_star", options.c_star);

    if max(Pc) > 800
        warning('Maximum chamber pressure (Pc = %.2f psi) exceeds the 800 psi limit.', max(Pc));
    end

    % Remove the last value from vectors to prevent NaN value in impulse calculation
    Pc = Pc(1:end-1);
    t = t(1:end-1);

    % Pe should be a vector, calculated for each value of Pc
    Pe = Pc ./ (1 + (gamma - 1)./2 .* Me.^2).^(gamma./(gamma-1));

    % Pressure ratio Pe/Pc for each time step
    pressure_ratio = Pe ./ Pc;

    cf_v = sqrt((2 .* gamma.^2 ./ (gamma - 1)) .* ((2 ./ (gamma + 1)).^((gamma + 1)./(gamma - 1))) .* ...
         (1 - pressure_ratio.^((gamma - 1)./gamma))) ...
         + (pressure_ratio).*epsilon;

    F_v = cf_v .* Pc .* At;
    F_v = F_v * 4.44822; % To Newtons

    m_dot = options.g .* Pc .* At ./ options.c_star;
    m_dot = m_dot * 0.453592; % To kg/s
    
    thrust_curve = [t', F_v'];
    mass_flow_curve = [t', m_dot'];

    if options.displayCurves

        % plot thrust curve
        figure()
        tiledlayout(3,1);

        nexttile
        plot(t, F_v)
        title('Thrust vs. Time')
        xlabel('Time (s)')
        ylabel('Thrust (N)')
        grid on;

        % plot mass flow curve
        nexttile
        plot(t, m_dot)
        title('Mass Flow Rate vs. Time')
        xlabel('Time (s)')
        ylabel('Mass Flow Rate (kg/s)')
        grid on;

        % plot chamber pressure curve
        nexttile
        plot(t, Pc)
        title('Chamber Pressure vs. Time')
        xlabel('Time (s)')
        ylabel('Chamber Pressure (psi)')
        grid on;
    end

end