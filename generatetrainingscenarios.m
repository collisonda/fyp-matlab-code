function [Scenarios] = generatetrainingscenarios()
%GENERATETRAININGSCENARIOS Creates all potential crossing scenarios for each number
%of agents. This provides a good set of training data.
%       Author: Dale Collison

theta = linspace(-pi/2,2*pi - pi/2, 9);
R = 90;
x = round(R * cos(theta),3);
y = round(R * sin(theta),3);
x = x(2:end-1);
y = y(2:end-1);

Scenarios = cell(63,1);
iScenario = 1;

%% nAgents = 2
for iPos = 1:length(x)
    positions(1,:) = [0, -90, 0, 90];
    positions(2,:) = [x(iPos), y(iPos), -x(iPos), -y(iPos)];
    Scenarios{iScenario,1} = positions;
    iScenario = iScenario + 1;
end

%% nAgents = 3
for iPos = 1:length(x)
    positions(1,:) = [0, -90, 0, 90];
    positions(2,:) = [x(iPos), y(iPos), -x(iPos), -y(iPos)];
    for jPos = 1:length(x)
        if ~(jPos <= iPos)
            positions(3,:) = [x(jPos), y(jPos), -x(jPos), -y(jPos)];
            Scenarios{iScenario,1} = positions;
            iScenario = iScenario + 1;
        end
    end    
end

%% nAgents = 4
for iPos = 1:length(x)
    positions(1,:) = [0, -90, 0, 90];
    positions(2,:) = [x(iPos), y(iPos), -x(iPos), -y(iPos)];
    for jPos = 1:length(x)
        if ~(jPos <= iPos)
            positions(3,:) = [x(jPos), y(jPos), -x(jPos), -y(jPos)];
            for kPos = 1:length(x)
                if ~(kPos <= jPos)
                    positions(4,:) = [x(kPos), y(kPos), -x(kPos), -y(kPos)];
                    Scenarios{iScenario,1} = positions;
                    iScenario = iScenario + 1;
                end
            end
        end
    end    
end

end
