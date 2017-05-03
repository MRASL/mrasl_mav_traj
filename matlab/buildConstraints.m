function [ Aeq, beq] = buildConstraints( n, r, constraints, t)
%BUILDCONSTRAINTS Builds the constraints matrix
% Inputs:
%   n           Order of the polynomials of a trajectory
%   r           Order of the derivative of the position
%   constraints r by wps+1 Matrix of constraints
%               Rows are derivatives and columns are waypoints
%               Inf denotes unfixed variable.
%   t           Time vector, always starts with 0
%
% Outputs:
%   H           H matrix for the QP problem

% Author:   Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
% 
% Basically follow equations 16 to 21 of richter_rss13_workshop
n_coeffs = n+1;
s = size(constraints);
n_wps = s(2);
coeffs = getCoefficientMatrix(n, r);
I = rot90(eye(n_coeffs, n_coeffs));     % Just need this for computations

A_0 = [];
b_0 = [];

for wp = 1:n_wps      % For each waypoint including initial conditions
    for der = 1:r       % For each derivative of the polynomial up to r-1
        if constraints(der, wp) ~= Inf    % We have a constraint
            if wp == 1
                % Initial conditions
                % Add only departure constraints
                a = zeros(1, (n_wps-1) * n_coeffs);
                % Set the part of 'a' corresponding to the wp
                % XXX
                int_t = 1 / (t(wp+1) - t(wp))^(der-1); % der-1 to get 0th derivative
                polynomial = coeffs(der, :) .* I(der, :) * int_t;
                % Get start idx of a block of coefficients
                idx = (wp-1) * n_coeffs + 1;
                a(1, idx:idx+n) = polynomial;
                b = constraints(der, wp);
            elseif wp == n_wps
                % Final conditions
                % Add only arrival constraints
                a = zeros(1, (n_wps-1) * n_coeffs);
                % Set the part of 'a' corresponding to the wp
                % XXX
                int_t_next = 1 / (t(wp) - t(wp-1))^(der-1); % t now and t prev
                polynomial = coeffs(der, :) * int_t_next;
                % Get the start idx of the **previous** block.
                idx = (wp-2) * n_coeffs + 1;
                a(1, idx:idx+n) = polynomial;
                b = constraints(der, wp);
            else
                % Middle/waypoint conditions
                a = zeros(2, (n_wps-1) * n_coeffs);

                % Add departure constraint
                int_t_next = 1 / (t(wp) - t(wp-1))^(der-1);  % time now and prev
                poly = coeffs(der, :) * int_t_next;
                idx = (wp-2) * n_coeffs + 1;
                a(1, idx:idx+n) = poly;
                
                % Add arrival constraint
                int_t = 1 / (t(wp+1) - t(wp))^(der-1);  % first row is 0th derivative
                poly = coeffs(der, :) .* I(der, :) * int_t;
                idx = (wp-1) * n_coeffs + 1;
                a(2, idx:idx+n) = poly;
                
                b = ones(2, 1);
                b = b .* constraints(der, wp); % both lines equal to this
            end
            A_0 = [A_0; a];
            b_0 = [b_0; b];
        end
    end
end
% 
% Aeq = A_0;
% beq = b_0;

% Now build continuity constraints follow equation 3.53 of "Control, 
% estimation, and planning algorithms for aggressive flight using onboard 
% sensing" By Bry, Adam Parker

% Basically what we will do is add arrival and derparture constraints on
% the same line to enforce continuity between two polynomials
A_t = [];
b_t = [];

for wp = 2:n_wps-1      % XXX for each INTERMEDIATE waypoint
    for der = 1:r+1       % for each dervative including the 0th derivative
        if constraints(der, wp) == Inf
            a = zeros(1, (n_wps-1) * n_coeffs);
            int_t = 1 / (t(wp) - t(wp-1))^(der-1);
            int_t_next = 1 / (t(wp+1) - t(wp))^(der-1);

            % from prev wp
            a_prev = coeffs(der, :) * int_t;
            idx_prev = (wp-2) * n_coeffs + 1;
            a(1, idx_prev:idx_prev+n) = a_prev;

            % to next wp
            a_next = - coeffs(der, :) .* I(der,:) * int_t_next;
            idx_next = (wp-1) * n_coeffs + 1;
            a(1, idx_next:idx_next+n) = a_next;

            b = 0;

            A_t = [A_t; a];
            b_t = [b_t; b];
        end
    end
end
% Acont = A_t;
% bcont = b_t;
Aeq = [A_0; A_t];
beq = [b_0; b_t];
end

