function [ stop ] = savetrajfun( T, optimValues, state )
%SAVETRAJFUN Summary of this function goes here
%   Detailed explanation goes here
stop = false;

global solutions;
i = optimValues.iteration + 1;

[solution.cost, solution.polynomial, solution.discrete, solution.der_discrete] ... 
    = computeTraj(segment2time(T));
solutions{i} = solution;
end

