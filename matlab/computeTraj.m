function [ total_cost, varargout ] = computeTraj( t )
%COMPUTETRAJ Summary of this function goes here
%   Detailed explanation goes here

global k_r;
global mu_r;
global n;
global m;
global states;
global w;

n_coeffs = n + 1;
H = zeros(n_coeffs * m, n_coeffs * m, states);

for i = 1:states
    H(:,:,i) = buildh(n, m, mu_r, k_r, t);
    [Aeq{i}, beq{i}] = buildConstraints(n, k_r, w(:,:,i), t);
end

solution = zeros(n_coeffs * m, states); 
options = optimoptions('quadprog', 'Display', 'off', 'MaxIterations', 4000);
total_cost = 0;
for i = 1:states
    [solution(:, i), fval] = quadprog(H(:,:,i), [], [], [], Aeq{i}, beq{i}, [], [], [], options);
    %fprintf('fval: %d \n', fval);
    total_cost = total_cost + fval;
end

% Additional args
n_addargs = max(nargout,1) - 1;
if n_addargs > 0
    varargout{1} = solution;
end

if n_addargs > 1
    [varargout{2}, varargout{3}] = discretizeTrajectory(solution, n, m, states, 0.01, t, 1, false);
end

