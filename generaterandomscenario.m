function Scenario = generaterandomscenario()
%GENERATERANDOMSCENARIO Generates a scenario with a random amount of agents
%at random positions.
%       Author: Dale Collison

maxAgents = 4;

xLim = [-100 100];
yLim = [-100 100];

nAgents = round((maxAgents-2)*rand)+2;
% nAgents = 4;
positions = zeros(nAgents,4);

nOverlaps = nAgents;

r = AgentConstants.RADIUS * 8;

while nOverlaps > 0
    overlaps = zeros(1,nAgents);
    for iAgent = 1:nAgents
        x = xLim(1) + 2*xLim(2)*rand;
        y = yLim(1) + 2*yLim(2)*rand;
        positions(iAgent,:) =  [x,y,-x,-y];
    end
    
    for iAgent = 1:nAgents
        for jAgent = 1:nAgents
            if iAgent ~= jAgent
                [X,~] = circcirc(positions(iAgent,1),positions(iAgent,2),r,positions(jAgent,1),positions(jAgent,2),r);
                if ~isnan(X)
                    overlaps(iAgent) = 1;
                end
            end
        end
    end
    
    nOverlaps = nnz(overlaps);
    
    
end

Scenario = positions;

end
