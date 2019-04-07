function [Scenarios] = generatescenarios()
clear
theta = linspace(-pi/2,2*pi - pi/2, 9);
R = 90;
x = round(R * cos(theta),3);
y = round(R * sin(theta),3);
x = x(2:end-1);
y = y(2:end-1);
r = AgentConstants.RADIUS * 5;
Scenarios = {};


for iPos = 1:length(x)
    positions(1,:) = [0, -90, 0, 90];
    positions(2,:) = [x(iPos), y(iPos), -x(iPos), -y(iPos)];
    Scenarios{end+1,1} = positions;
end

for iPos = 1:length(x)
    positions(1,:) = [0, -90, 0, 90];
    positions(2,:) = [x(iPos), y(iPos), -x(iPos), -y(iPos)];
    for jPos = 1:length(x)
        if ~(jPos <= iPos)
            positions(3,:) = [x(jPos), y(jPos), -x(jPos), -y(jPos)];
            Scenarios{end+1,1} = positions;
        end
    end
    
end

for iPos = 1:length(x)
    positions(1,:) = [0, -90, 0, 90];
    positions(2,:) = [x(iPos), y(iPos), -x(iPos), -y(iPos)];
    for jPos = 1:length(x)
        if ~(jPos <= iPos)
            positions(3,:) = [x(jPos), y(jPos), -x(jPos), -y(jPos)];
            for kPos = 1:length(x)
                if ~(kPos <= jPos)
                    positions(4,:) = [x(kPos), y(kPos), -x(kPos), -y(kPos)];
                    Scenarios{end+1,1} = positions;
                end
            end
        end
    end
    
end

end
