function [S] = createstatespace()
%% Create state space
[x,y] = createarc(-AgentConstants.VISION_ANGLE+pi/2,AgentConstants.VISION_ANGLE+pi/2,0,0,AgentConstants.NEIGHBOURHOOD_RADIUS);
S = cell(13,23);
for i = 1:13
    for j = 1:23
        S{i,j} = [5*j-60, 65-(5*i)];
        if ~inpolygon(S{i,j}(1),S{i,j}(2),x,y)
            S{i,j} = [];
        end
    end
end
end

