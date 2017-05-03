function [ t ] = trealloc( t, delta )
%TREALLOC Reallocate time for each segments, assume provided delta doesn't
%change total time
original_last = t(end);

T = time2segment(t);
T = T + delta;
t = segment2time(T);

assert(t(end) == original_last);

end

