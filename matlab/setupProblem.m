function setupProblem()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Filename:   setupProblem.m
%   Author:     Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
%   Class:      MTH8408
%   Description:Setup all the global variables required to describe the
%   minimum snap optimization problem.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global k_r;
global mu_r;
global n;
global m;
global states;
global h;
global w;
global t0;
global save_intermediate_solutions;

k_r = 4;
mu_r = 1;
n = 6;      % Order of the polynomials describing the trajectory
m = 3;      % Number of waypoints (not including initial conditions)
states = 3;
h = 0.001;

w = zeros(k_r+1, m+1, states);
% arc
% Waypoint constraints
% X axis
w(:, :, 1) = [  0   1   1   0; ...
                0   Inf Inf 0; ...  % velocity constraints
                0   Inf Inf 0; ...  % acceleration constraints
                0   Inf Inf 0; ...  % jerk constraints
                0   Inf Inf 0];     % snap constraints
            
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

t0 = [0 0.5 2.5 3];

% Slalom
% w(:, :, 1) = [  0 1   0   -1  0   1   0; ...
%                 0 Inf Inf Inf Inf Inf 0; ...  % velocity constraints
%                 0 Inf Inf Inf Inf Inf 0; ...  % acceleration constraints
%                 0 Inf Inf Inf Inf Inf 0; ...  % jerk constraints
%                 0 Inf Inf Inf Inf Inf 0];     % snap constraints
%             
% % Y axis
% w(:, :, 2) = [  0 1   2   3   4   5   6; ...
%                 0 Inf Inf Inf Inf Inf 0; ...  % velocity constraints
%                 0 Inf Inf Inf Inf Inf 0; ...  % acceleration constraints
%                 0 Inf Inf Inf Inf Inf 0; ...  % jerk constraints
%                 0 Inf Inf Inf Inf Inf 0];     % snap constraints
%             
% % Z axis
% w(:, :, 3) = [  1.5 1   2   1   2   1   1.5; ...
%                 0   Inf Inf Inf Inf Inf 0; ...  % velocity constraints
%                 0   Inf Inf Inf Inf Inf 0; ...  % acceleration constraints
%                 0   Inf Inf Inf Inf Inf 0; ...  % jerk constraints
%                 0   Inf Inf Inf Inf Inf 0];     % snap constraints
% 
% t0 = [0 1 2 3 4 5 6];

% figure 8
% w(:, :, 1) = [  0 2   4   2   0   -2  -4  -2  0; ...
%                 0 Inf Inf Inf Inf Inf Inf Inf 0; ...  % velocity constraints
%                 0 Inf Inf Inf Inf Inf Inf Inf 0; ...  % acceleration constraints
%                 0 Inf Inf Inf Inf Inf Inf Inf 0; ...  % jerk constraints
%                 0 Inf Inf Inf Inf Inf Inf Inf 0];     % snap constraints
%             
% % Y axis
% w(:, :, 2) = [  0   -2  0   2   0   -2  0   2   0; ...
%                 0   Inf Inf Inf Inf Inf Inf Inf 0; ...  % velocity constraints
%                 0   Inf Inf Inf Inf Inf Inf Inf 0; ...  % acceleration constraints
%                 0   Inf Inf Inf Inf Inf Inf Inf 0; ...  % jerk constraints
%                 0   Inf Inf Inf Inf Inf Inf Inf 0];     % snap constraints
%             
% % Z axis
% w(:, :, 3) = [  1   1.5 2   1.5 1   1.5 2   1.5 1; ...
%                 0   Inf Inf Inf Inf Inf Inf Inf 0; ...  % velocity constraints
%                 0   Inf Inf Inf Inf Inf Inf Inf 0; ...  % acceleration constraints
%                 0   Inf Inf Inf Inf Inf Inf Inf 0; ...  % jerk constraints
%                 0   Inf Inf Inf Inf Inf Inf Inf 0];     % snap constraints
% 
% t0 = [0 1 2 3 4 5 6 7 8];

% 
save_intermediate_solutions = true;
            
end