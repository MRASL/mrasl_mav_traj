%% Test file to see how to compute the jacobian of the constraints matrix
clear;

global k_r;
global k_psi;
global mu_r mu_psi n m states;
k_r = 4;    % Order of derivative of the position
k_psi = 2;  % Order of derivative of the yaw
mu_r = 1;   % Non-dimentionalization constant for the position integral
mu_psi = 1; % Non-dimentionalization constant for the yaw integral
n = 6;      % Order of the polynomials describing the trajectory
m = 3;      % Number of waypoints (not including initial conditions)
states = 3;

% For a quadratic optimization problem of the form
%   min c'Hc + f'c
% subject to
%       A * c <= b
%       Aeq * c = beq
% We know the linear term f' is nul and c is a 4nm x 1 vector containing the
% coefficients of the polynomials. So now we have to build the matrix H.
% Note: the numbering for n starts at 0 so the actual size of c is
% 4*(n+1)*m x 1

% Time constraints
t = [0 1 2 3];
%t = [0 0.5 2.5 3];

% Waypoint constraints
% X axis
w(:, :, 1) = [  0   1   1   0; ...
                0 Inf Inf 0; ...  % velocity constraints
                0 Inf Inf 0; ...  % acceleration constraints
                0 Inf Inf 0; ...  % jerk constraints
                0 Inf Inf 0];     % snap constraints
            
% Y axis
w(:, :, 2) = [  0   0   2   2; ...
                0   Inf Inf 0; ...  % velocity constraints
                0   Inf Inf 0; ...  % acceleration constraints
                0   Inf Inf 0; ...  % jerk constraints
                0   Inf Inf 0];     % snap constraints
            
% Z axis
w(:, :, 3) = [  1.5 1.5 1.5 1.5; ...
                0   Inf Inf 0; ...  % velocity constraints
                0   Inf Inf 0; ...  % acceleration constraints
                0   Inf Inf 0; ...  % jerk constraints
                0   Inf Inf 0];     % snap constraints
            
% Yaw
w(:, :, 4) = [  0   0   0   0; ...
                0   Inf Inf 0; ...  % angular velocity constraints
                0   Inf Inf 0; ...  % angular acceleration constraints
                0   Inf Inf 0; ...  % jerk constraints
                0   Inf Inf 0];     % snap constraints

% Before starting make sure the developper didn't do anything stupid...
assert(length(t) == m+1);
for state = 1:states
    s = size(w(:,:,state));
    assert(s(1) == k_r+1);
    assert(s(2) == m+1);
end
            
n_coeffs = n + 1;
s = size(w);
n_constraints = s(1) * s(2);
% Aeq = zeros(n_constraints, n_coeffs * m, states);
% beq = zeros(n_constraints, 1, states);

for i = 1:states
    [Aeq{i}, beq{i}] = buildConstraints(n, k_r, w(:,:,i), t);
end


%% Start jacobian stuff
A = Aeq{i};