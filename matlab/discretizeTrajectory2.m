function [ traj, time ] = discretizeTrajectory2(  coeffs, n, m, states, dt, arrivalTimes, alpha)

global k_r;

n_coeffs = n+1;

% Slice up coefficients into real polynomials
polynomial = zeros(n_coeffs, m, k_r+1, states);
for state = 1:states
    polynomial(:,:, 1, state) = reshape(coeffs(:,state), n_coeffs, m);
    scale_vec = ((1/alpha).^(n:-1:0))';
    polynomial(:,:, 1, state) = scale_vec(:, ones(1,m)) .* polynomial(:,:, 1, state);
    %  .* ;
    for der = 1:k_r
        for wp = 1:m
            poly_temp = polynomial(:, wp, der, state);  % Get the polynomial
            poly_temp = polyder(poly_temp);             % Derive it
             % Pad zeros to the left
            poly_temp = [zeros(1, n_coeffs-length(poly_temp)) poly_temp];
            poly_temp = transpose(poly_temp);
            polynomial(:, wp, der+1, state) = poly_temp;
        end
    end
end

% Basically the polynomials are nondimentionalized, that means they are valid
% between 0 and 1. We have to map these solutions to the real departure and
% arrival times corresponding to our constraints. We can further scale our
% solutions in time using an alpha term.

time = 0:dt:arrivalTimes(end);
traj = zeros(length(time), k_r+1, states);
for state = 1:states
    for t = 1:length(time)
        % Find which trajectory segment were on
        if time(t) == 0
            wp = 1;
        else
            wp = find(time(t)>arrivalTimes, 1, 'last');
        end
        for der = 1:k_r+1
            scaledt = (time(t) - arrivalTimes(wp)) / (arrivalTimes(wp+1) - arrivalTimes(wp));
            scaledt = scaledt * alpha;
            poly_temp = polynomial(:, wp, der, state);
            traj(t, der, state) = polyval(poly_temp, scaledt);
        end    
    end
end

