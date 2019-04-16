function [S] = createstatespace()
%CREATESTATESPACE Creates a cell array containing all states that an agent
%can be in. A state is defined by the position of its closest neighbour
%relative to the goal position.
%       Author: Dale Collison

[x,y] = createcircle(0,0,AgentConstants.NEIGHBOURHOOD_RADIUS);
S = cell(13,13);
for i = 1:13
    for j = 1:13
        S{i,j} = [10*j-70, 70-(10*i)];
        if ~inpolygon(S{i,j}(1),S{i,j}(2),x,y)
            S{i,j} = [NaN, NaN];
        end
    end
end

end
