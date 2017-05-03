function [ T ] = time2segment( t )
%time2segment Takes a vector of arrival times in input (first element is
%always 0) and returns a vector of segment times in output.
assert(t(1) == 0);

T = t(2:end) - t(1:end-1);

end

