function [Scenarios] = generatescenarios(nScenarios)
Scenarios = cell(1,nScenarios);
r = 90;
th = linspace(-pi/2,2*pi-pi/2,nScenarios+2);
xunit = round(2*(r*cos(th)))/2;
yunit = round(2*(r*sin(th)))/2;

for iScenario = 1:nScenarios
    data = [xunit(iScenario+1), yunit(iScenario+1); xunit(1),yunit(1);...
        2*r/AgentConstants.MAX_SPEED, 2*r/AgentConstants.MAX_SPEED];
    Scenarios{iScenario} =  data;
end

% Scenarios = [Scenarios(1), Scenarios(5), Scenarios(2), Scenarios(6),...
%     Scenarios(3), Scenarios(7), Scenarios(4)];

end

