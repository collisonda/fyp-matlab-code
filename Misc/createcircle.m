function [xunit,yunit] = createcircle(x,y,r)
%CREATECIRCLE Creates a circle with a given centre and radius.

theta = 0:pi/50:2*pi;
% theta = linspace(0,2*pi,25);
xunit = r * cos(theta) + x;
yunit = r * sin(theta) + y;
end

