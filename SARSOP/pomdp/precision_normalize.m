% Authors: Jared
% When writing to file, a vector may not be normalized to the written 
% precision. This function truncates the vector and renormalizes
function [x] = precision_normalize(x)
    x = x/sum(x);
    x = round(x*1000)/1000;
    temp = sum(x);
    diff = 1-temp;
    idx = find(x==max(x));
    x(idx(1)) = x(idx(1))+diff;
end

