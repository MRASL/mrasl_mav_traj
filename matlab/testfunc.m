function [ n, varargout ] = testfunc( )
n = max(nargout,1) - 1;
for i = 1:n
    varargout{i} = rand();
end
end

