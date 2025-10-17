function thrust_curve = get_thrust_curve(r, Me, epsilon, gamma, options)
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
    end

    At = pi * options.Dt^2 / 4; % [in^2]

    [t, Pc, ~] = project_grain(r, "Dt", options.Dt, "c_star", options.c_star);

    % Pe should be a vector, calculated for each value of Pc
    Pe = Pc ./ (1 + (gamma - 1)./2 .* Me.^2).^(gamma./(gamma-1));

    % Pressure ratio Pe/Pc for each time step
    pressure_ratio = Pe ./ Pc;

    cf_v = sqrt((2 .* gamma.^2 ./ (gamma - 1)) .* ((2 ./ (gamma + 1)).^((gamma + 1)./(gamma - 1))) .* ...
         (1 - pressure_ratio.^((gamma - 1)./gamma))) ...
         + (pressure_ratio).*epsilon;

    % Remove the last value from vectors to prevent NaN value in impulse calculation
    cf_v = cf_v(1:end-1);
    Pc = Pc(1:end-1);
    t = t(1:end-1);

    F_v = cf_v .* Pc .* At;
    
    thrust_curve = [t', F_v'];

    % plot thrust curve
    figure()
    plot(t, F_v)
    title('Thrust vs. Time')
    xlabel('Time (s)')
    ylabel('Thrust (lbf)')
    grid on;


end