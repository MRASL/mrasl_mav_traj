function [ cost, varargout ] = objective(T)
addpath('helpers');
% [cost]
% [cost, gradient]

global m;
global h;

t = segment2time(T);
[cost] = computeTraj(t);

% Additional args
n_addargs = max(nargout,1) - 1;
if n_addargs == 1
    % compute gradient
    gi = eye(m);                % m can also be seen as the number of segments
    gi(gi==0) = -1 / (m+1-2);   % m+1 = num keyframes
    nabla_gi_f = zeros(m, 1);
    for i = 1:m
        Ti = T + h * gi(:,i)';
        t = segment2time(Ti);
        [cost_Thgi] = computeTraj(t);
        nabla_gi_f(i) = (cost_Thgi - cost) / h;
    end
    varargout{1} = nabla_gi_f;    
end

end

