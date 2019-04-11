function [A] = createactionspace()
%CREATEACTIONSPACE Creates a cell array containing all possible agent
%velocities.
%       Author: Dale Collison

A = cell(51,51);
x = linspace(-6,6,51);
y = linspace(6,-6,51)';

for iY = 1:length(y)
    for iX = 1:length(x)
        if norm([x(iX), y(iY)]) > 6
            A{iY,iX} = [NaN, NaN];
        else
        A{iY,iX} = [x(iX), y(iY)];
        end
    end    
end
