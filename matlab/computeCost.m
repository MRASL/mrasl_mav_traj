function [ cost ] = computeCost( i, t )
%COMPUTECOST Summary of this function goes here
%   Detailed explanation goes here

global k_r;
global mu_r;
global n;
global m;
global states;
global solutions;
global w;

if i == 0
    for s = 1:states
        H(:,:,s) = buildh(n, m, mu_r, k_r, t);
        [Aeq{s}, beq{s}] = buildConstraints(n, k_r, w(:,:,s), t);
    end
    
    solution = zeros((n+1) * m, states); 
    options = optimoptions('quadprog', 'Display', 'off', 'MaxIterations', 4000);
    %cost = 0;
    for s = 1:states
        solution(:, s) = quadprog(H(:,:,s), [], [], [], Aeq{s}, beq{s}, [], [], [], options);
        %cost = cost + (transpose(solution(:,s)) * H(:,:,s) * solution(:,s));
    end
    
    cost = 0;
    for i = 1:states
        cost = cost + (transpose(solution(:,i)) * H(:,:,i) * solution(:,i));
    end
else
    cost = 0;
    solution = solutions{i}.polynomial;
    for s = 1:states
        H(:,:,s) = buildh(n, m, mu_r, k_r, t);
        x = solution(:,s);
        cost = cost + (transpose(x) * H(:,:,s) * x);
    end
   
end

end

