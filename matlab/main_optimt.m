addpath('helpers');
clear all;
setupProblem();

global t0;

T0 = time2segment(t0);

Aeq = ones(1, length(T0));
beq = t0(end);
lb = zeros(1, length(T0));

tic
global save_intermediate_solutions;
if save_intermediate_solutions
    options = optimoptions('fmincon', ...
                            'SpecifyObjectiveGradient',true, ...
                            'Display', 'iter-detailed', ...
                            'OutputFcn', @savetrajfun);
else
    options = optimoptions('fmincon', ...
                            'SpecifyObjectiveGradient',true, ...
                            'Display', 'iter-detailed');
end
T = fmincon(@objective, T0, [], [], Aeq, beq, lb, [], [], options);
t_final = segment2time(T);
exectime = toc;
fprintf('Execution time %4.2f ms\n', exectime*1000);

%% Find polynomial solution corresponding to T
global solutions;
cost = computeCost(0, t0);
fprintf('Initial cost %4.2f\n', cost);
cost = computeCost(length(solutions), t_final);
fprintf('Best cost    %4.2f\n', cost);


%% Plot solutions
%global solutions;
global m;
iters = length(solutions);

close all;
figure
hold on;
handles = [];
for i = 1:iters
    x = solutions{i}.discrete(:,1);
    y = solutions{i}.discrete(:,2);
    z = solutions{i}.discrete(:,3);
    l = length(x) / m;
%     plot(x(1:l), y(1:l), '-r');
%     plot(x(l+1:2*l), y(l+1:2*l), '-g');
%     plot(x(2*l+1:3*l), y(2*l+1:3*l), '-b');
    if i == 1
        h(i) = plot3(x, y, z, '--');
    elseif i == iters
        h(i) = plot3(x, y, z, '.');
    else
        h(i) = plot3(x,y,z);
    end
end
global w;
h(iters+1) = plot3(w(1,:,1), w(1,:,2), w(1,:,3), 'd');
legend(h([1 iters iters+1]), 'initial guess', 'final solution', 'waypoints');
grid on;
axis equal;
xlabel('x (m)');
ylabel('y (m)');
% 
figure
title('Fonction de coût');
hold on;
for i = 1:iters
    cost = solutions{i}.cost;
    plot(i, cost, 'd', ...
        'MarkerEdgeColor','k',...
    'MarkerFaceColor',[.49 1 .63],...
    'MarkerSize',7);
end
grid on
xlabel('iterations');
ylabel('coût');
