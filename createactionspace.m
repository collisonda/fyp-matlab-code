function [A] = createactionspace()
%% Create action space
X = -AgentConstants.MAX_SPEED:0.5:AgentConstants.MAX_SPEED;
Y = -AgentConstants.MAX_SPEED:0.5:AgentConstants.MAX_SPEED;
A = cell(length(X),length(Y));
for iX = 1:length(X)
    for iY = 1:length(Y)
        x = X(iX);
        y = X(iY);
        if norm([x,y]) <= AgentConstants.MAX_SPEED
           A{iX,iY} = [x,y];
        end
    end
end
end

