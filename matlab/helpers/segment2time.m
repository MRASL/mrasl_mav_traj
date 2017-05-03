function [ t ] = segment2time( T )
%segment2time Takes in input a series of time segments and returns a
%vector of arrival times (with the first element being 0)
t = [0 cumsum(T)];

end
