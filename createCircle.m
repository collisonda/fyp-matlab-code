function [xunit,yunit] = createCircle(x,y,r)
    th = 0:pi/100:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
end