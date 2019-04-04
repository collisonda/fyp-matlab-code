function [A] = createactionspace()
%% Create action space
% [x,~] = pol2cart(linspace(0, 2*pi, 201), AgentConstants.MAX_SPEED);
% [~,y] = pol2cart(linspace(pi/2, 2*pi+pi/2, 201), AgentConstants.MAX_SPEED);
% x = round(-x(1:(length(x)+1)/2),3);
% y = round(y(1:(length(y)+1)/2),3);
% A = cell(length(y),length(x));
% for iY = 1:length(y)
%     for iX = 1:length(x)
%         if norm([x(iX),y(iY)]) > AgentConstants.MAX_SPEED
%             A{iY,iX} = [NaN, NaN];
%         else
%             A{iY,iX} = [x(iX), y(iY)];
%         end
%     end
% end

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


