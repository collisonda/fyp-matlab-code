function [xunit,yunit] = createcircle(x,y,r)
%CREATECIRCLE Summary of this function goes here
%   Detailed explanation goes here
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
end

