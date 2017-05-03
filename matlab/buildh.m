function [ H ] = buildh( n, wps, mu, r, t )
%BUILDH Builds the H matrix for a single segment of the optimization
%problem
% Inputs:
%   n           Order of the polynomials of a trajectory
%   wps         Number of waypoints (not including initial conditions)
%   mu          Constant making the integrand non-dimentional
%   r           Order of the derivative of the position
%   t           Time vector, always starts with 0
%
% Outputs:
%   H           H matrix for the QP problem

% Author:   Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
% Good reference paper
% Adam Bry's thesis and richter_rss13_workshop.

num_coeffs = n + 1;

% Follow equations 3.30 page 37
H = [];
for wp = 1:wps
    H_r = zeros(num_coeffs);
    t0 =  t(wp);
    tend = t(wp+1);
    for i = 0:n
        for l = 0:n
            if i >= r && l >=r
                cum_mul = 1;
                for m = 0:r-1
                    cum_mul = cum_mul * (i-m) * (l-m);
                end
                H_r(i+1,l+1) = cum_mul;% * (tend-t0)^(i+l-2*r+1);
                H_r(i+1,l+1) = H_r(i+1,l+1) / (i+l-2*r+1);
            else
                H_r(i+1,l+1) = 0;
            end
        end
    end
    
    % "The cost matrix is constructed as block diagonal on Qk" (page 41)
    H_r =  1 ./ ((t(wp+1) - t(wp))^(2*r-1)) .* H_r;
    H_r = rot90(rot90(H_r));
    H = blkdiag(H, H_r);
end

end

