close all;
clear;
addpath('../../../../Minimum-snap-trajectory-generator/');

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
            
w1 = w;
w1(w1==Inf) = 255;
t = [0 1 3 4];

% mine
H = buildh(6, 3, 1, 4, t);
[Aeq, beq] = buildConstraints(6, 4, w(:,:,1), t);
Acont = Aeq(13:end, :);
bcont = beq(13:end, :);
Aeq = Aeq(1:12, :);
beq = beq(1:12, :);

% his
H_joint = [];
for i=1 : 3
    % find cost matrix for each segment
    H1 = findCostMatrix(6, 4); 
    % multiply by time factor to nondimensionalize
    H1 = 1 ./ ((t(i+1) - t(i))^(2*4-1)) .* H1;
    % put in block diagonal matrix
    H_joint = blkdiag(H_joint, H1);
end
[Aeq1, beq1] = findFixedConstraints(4, 6, 3, 1, w1, t');
[Acont1, bcont1] = findContConstraints(4, 6, 3, 1, w1, t');

if isequal(Acont, Acont1) && isequal(Aeq ,Aeq1) ...
        && isequal(beq ,beq1) ...
        && isequal(bcont ,bcont1)
    fprintf('success\n');
else
    fprintf('fail\n');
end

if isequal(H, H_joint)
    fprintf('h success\n');
else
    fprintf('h fail\n');
end