function [ coefficients ] = getCoefficientMatrix( n, r )
%GETCOEFFICIENTMATRIX Gives a matrix where each line are the coefficients
%of a derivative of a polynomial of order n
% Inputs:
%   n           Order of the polynomials of a trajectory
%   r           Order of the last derivative
% Example:
%   A polynomial of order n = 6 and we want the coefficients up to
%   derivative r = 4. We will get
%
% coefficients =
% 
%      1     1     1     1     1     1     1
%      6     5     4     3     2     1     0
%     30    20    12     6     2     0     0
%    120    60    24     6     0     0     0
%    360   120    24     0     0     0     0

% Author:   Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>

num_coeffs = n + 1;     % +1 to include the c_0 coefficient
num_poly = r + 1;       % +1 to include the 0th derivative

coefficients = zeros(num_poly, num_coeffs);
polynomial = ones(1, num_coeffs);
for i = 1:num_poly
    coefficients(i,1:length(polynomial)) = polynomial;
    polynomial = polyder(polynomial);
end

end