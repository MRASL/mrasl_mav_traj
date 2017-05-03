%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Filename:   main.m
%   Author:     Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
%   Class:      MTH8408
%   Description:Implementation of Minimum snap trajectory generation but
%   modified to follow the formulation by Adam Bry/Charles Richter. The
%   only difference is that the flat outputs are each treated as individual
%   optimization problems, so we have multiple problems to solve instead of
%   one big one.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
for trial = 1:1
clc;

global k_r;
global k_psi;
global mu_r mu_psi n m states;
k_r = 4;    % Order of derivative of the position
k_psi = 2;  % Order of derivative of the yaw
mu_r = 1;   % Non-dimentionalization constant for the position integral
mu_psi = 1; % Non-dimentionalization constant for the yaw integral
n = 6;      % Order of the polynomials describing the trajectory
m = 8;      % Number of waypoints (not including initial conditions)
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
t = [0 1 2 3 4 5 6 7 8];
%t = [0 0.5 2.5 3];

% Waypoint constraints
% X axis
%     wp1 << 0, 0, 1;
%     wp2 << 2, -2, 1.5;
%     wp3 << 4, 0, 2;
%     wp4 << 2, 2, 1.5;
%     wp5 << 0, 0, 1;
%     wp6 << -2, -2, 1.5;
%     wp7 << -4, 0, 2;
%     wp8 << -2, 2, 1.5;
%     wp9 << 0, 0, 1;
w(:, :, 1) = [  0 2   4   2   0   -2  -4  -2  0; ...
                0 Inf Inf Inf Inf Inf Inf Inf 0; ...  % velocity constraints
                0 Inf Inf Inf Inf Inf Inf Inf 0; ...  % acceleration constraints
                0 Inf Inf Inf Inf Inf Inf Inf 0; ...  % jerk constraints
                0 Inf Inf Inf Inf Inf Inf Inf 0];     % snap constraints
            
% Y axis
w(:, :, 2) = [  0   -2  0   2   0   -2  0   2   0; ...
                0   Inf Inf Inf Inf Inf Inf Inf 0; ...  % velocity constraints
                0   Inf Inf Inf Inf Inf Inf Inf 0; ...  % acceleration constraints
                0   Inf Inf Inf Inf Inf Inf Inf 0; ...  % jerk constraints
                0   Inf Inf Inf Inf Inf Inf Inf 0];     % snap constraints
            
% Z axis
w(:, :, 3) = [  1   1.5 2   1.5 1   1.5 2   1.5 1; ...
                0   Inf Inf Inf Inf Inf Inf Inf 0; ...  % velocity constraints
                0   Inf Inf Inf Inf Inf Inf Inf 0; ...  % acceleration constraints
                0   Inf Inf Inf Inf Inf Inf Inf 0; ...  % jerk constraints
                0   Inf Inf Inf Inf Inf Inf Inf 0];     % snap constraints
            
% Yaw
w(:, :, 4) = [  0   0   0   0   0   0    0   0   0; ...
                0   Inf Inf Inf Inf Inf  Inf Inf 0; ...  % angular velocity constraints
                0   Inf Inf Inf Inf Inf  Inf Inf 0; ...  % angular acceleration constraints
                0   Inf Inf Inf Inf Inf  Inf Inf 0; ...  % jerk constraints
                0   Inf Inf Inf Inf Inf  Inf Inf 0];     % snap constraints

% Before starting make sure the developper didn't do anything stupid...
assert(length(t) == m+1);
for state = 1:states
    s = size(w(:,:,state));
    assert(s(1) == k_r+1);
    assert(s(2) == m+1);
end
            
n_coeffs = n + 1;
H = zeros(n_coeffs * m, n_coeffs * m, states);
s = size(w);
n_constraints = s(1) * s(2);
% Aeq = zeros(n_constraints, n_coeffs * m, states);
% beq = zeros(n_constraints, 1, states);

for i = 1:states
    H(:,:,i) = buildh(n, m, mu_r, k_r, t);
    [Aeq{i}, beq{i}] = buildConstraints(n, k_r, w(:,:,i), t);
end

solution = zeros(n_coeffs * m, states); 
tic
options = optimoptions('quadprog', 'Display', 'iter-detailed', 'MaxIterations', 4000);
total_cost = 0;
for i = 1:states
    [solution(:, i), fval] = quadprog(H(:,:,i), [], [], [], Aeq{i}, beq{i}, [], [], [], options);
    %fprintf('fval: %d \n', fval);
    total_cost = total_cost + fval;
end
exec_time(trial) = toc;
fprintf('total cost: %5.2f \n', total_cost);
fprintf('execution time: %d \n', exec_time(trial)*1000);
end
value = 0;
for i = 1:states
    value = value + (transpose(solution(:,i)) * H(:,:,i) * solution(:,i));
end
fprintf('value %4.2f\n', value);
fprintf('avg execution time over %d trials: %4.2f ns\n', trial, mean(exec_time*1e9));

%%
close all;
alpha = 1;
dt = 0.1*alpha;
t = t .* alpha;
[traj, time] = discretizeTrajectory2(solution, n, m, states, dt, t, alpha);
figure

% generate scaled
alpha = 2;
dt = 0.1*alpha;
t = t .* alpha;
[traj_scaled, time_scaled] = discretizeTrajectory2(solution, n, m, states, dt, t, alpha);
%time = 0:dt:t(end);

iter = 1;
for der=1:k_r-1
    for state = 1:states
        subplot(k_r-1, states, iter);
        iter = iter + 1;
        plot(time, traj(:,der, state));
        hold on;
        plot(time_scaled, traj_scaled(:,der, state));
        grid on;
        switch state
            case 1
                s = ' x';
            case 2
                s = ' y';
            case 3
                s = ' z';
        end
        xlabel('time');
        switch der-1
            case 0
                title(strcat('position ', s));
                ylabel('m');
                if state == 1
                    legend('non scaled', 'scaled');
                end
            case 1
                title(strcat('velocity ', s));
                ylabel('m/s');
            case 2
                title(strcat('acceleration ',s));
                ylabel('m/s^2');
            case 3
                title(strcat('jerk ',s));
                ylabel('m/s^3');
            case 4
                title(strcat('snap ',s));
                ylabel('m/s^4');
        end
        xlim([0 16]);
    end
end
%set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.

% figure 
% set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
% plot3(traj(:,1,1), traj(:,1,2), traj(:,1,3));
% grid on;
% minor on;


% [traj trajder] = discretizeTrajectory(solution, n, m, states, 0.01, t, 1, true);
% 
% figure;
% time = 0:0.01:0.01*(length(trajder(:,1,1))-1);
% iter = 1;
% for der = 0:k_r
%     for state = 1:states
%         subplot(k_r+1, states, iter);
%         iter = iter + 1;
%         if der == 0
%             plot(time, traj(:, state));
%         else
%             plot(time, trajder(:,der,state));
%         end
%         switch state
%             case 1
%                 s = 'x';
%             case 2
%                 s = 'y';
%             case 3
%                 s = 'z';
%         end
%         xlabel('time');
%         switch der
%             case 0
%                 title(strcat('position ', s));
%             case 1
%                 title(strcat('velocity ', s));
%             case 2
%                 title(strcat('acceleration ',s));
%             case 3
%                 title(strcat('jerk ',s));
%             case 4
%                 title(strcat('snap ',s));
%         end                
%     end 
% end







