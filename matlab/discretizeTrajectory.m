function [ traj, varargout ] = discretizeTrajectory( coeffs, n, m, states, dt, t, alpha, do_plot)
%DISCRETIZETRAJECTORY Takes in the solution to the QP problem and
%discretizes it to plot the trajectory.
% Inputs:
%   coeffs      Solution to QP problem
%   n           Order of the polynomials of a trajectory
%   m           Number of waypoints
%   states      Number of states x, y, z and maybe psi
%   dt          Time step
%   times       Vector of the arrival times for each waypoint. Should
%               always 0 as the first element.
%   alpha       Time scale factor > 1 dilates and < 1 constricts
%   do_plot     Plot?

% Author:   Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>

global k_r;

if nargin < 7
    alpha = 1
end

if nargin < 8
    do_plot = false;
end
    
n_addargs = max(nargout,1) - 1;
do_der = false;
if n_addargs > 0
    do_der = true;
    ders = [];
end
n_coeffs = n + 1;
traj = [];

if do_plot
    figure
    hold on
end
for wp = 1:m
    %time = 0:dt:alpha;
    time = 0:dt:1;
    segment_samples = length(time);
    segment = zeros(segment_samples, states);
    if do_der
        der_segment = zeros(segment_samples, k_r, states);
    end
    for state = 1:states
        l_coeffs_idx = (wp-1) * n_coeffs + 1;
        h_coeffs_idx = l_coeffs_idx + n_coeffs - 1;
        segment_coeffs = coeffs(l_coeffs_idx:h_coeffs_idx, state);
        %segment_coeffs = segment_coeffs .* ((1/alpha).^(n_coeffs:-1:1))';
        segment(:, state) = polyval(segment_coeffs, time);
        if do_der
            time_scaled = 0:alpha*dt:alpha;
            der_coeff = segment_coeffs .* ((1/alpha).^(n_coeffs:-1:1))';            
            for der = 1:k_r
                der_coeff = polyder(der_coeff);
                der_segment(:, der, state) = polyval(der_coeff, time_scaled);
            end
        end
    end
    if do_plot
        plot3(segment(:, 1), segment(:, 2), segment(:, 3), 'o-');
        grid on
        grid minor
        %axis equal
    end
    traj = [traj; segment];
    if do_der
        ders = [ders; der_segment];
        
    end
end
varargout{1} = ders;
end
