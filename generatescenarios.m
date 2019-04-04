function [Scenarios] = generatescenarios()

iScenario = 1;
%% Scenario 1
agent1 = [0, -90, 0, 90];
agent2 = [0, 90, 0, -90];
Scenarios{iScenario} = [agent1; agent2];

iScenario = iScenario + 1;

%% Scenario 2
xy = 90*cos(pi/4);
agent1 = [-xy, -xy, xy, xy];
agent2 = [xy, -xy, -xy, xy];
Scenarios{iScenario} = [agent1; agent2];

iScenario = iScenario + 1;

%% Scenario 3
x = 90*cos(pi/3);
y = 90*sin(pi/3);
agent1 = [-x, -y, x, y];
agent2 = [x, -y, -x, y];
Scenarios{iScenario} = [agent1; agent2];

iScenario = iScenario + 1;

%% Scenario 4
x = 90*cos(pi/6);
y = 90*sin(pi/6);
agent1 = [-x, -y, x, y];
agent2 = [x, -y, -x, y];
Scenarios{iScenario} = [agent1; agent2];

iScenario = iScenario + 1;

%% Scenario 5
agent1 = [0, -90, 0, 90];
agent2 = [77.94, 45, -77.94, -45];
agent3 = [-77.94, 45, 77.94, -45];
Scenarios{iScenario} = [agent1; agent2; agent3];

iScenario = iScenario + 1;

%% Scenario 6
agent1 = [90, 0, -90, 0];
agent2 = [-90, 15, 90, 15];
agent3 = [-90, -15, 90, -15];
Scenarios{iScenario} = [agent1; agent2; agent3];

iScenario = iScenario + 1;

%% Scenario 7
agent1 = [0, -90, 0, 90];
agent2 = [-90, 0, 90, 0];
agent3 = [0, 90, 0, -90];
agent4 = [90, 0, -90, 0];
Scenarios{iScenario} = [agent1; agent2; agent3; agent4];

iScenario = iScenario + 1;

%% Scenario 8
agent1 = [90, 15, -90, 15];
agent2 = [90, -15, -90, -15];
agent3 = [-90, 15, 90, 15];
agent4 = [-90, -15, 90, -15];
Scenarios{iScenario} = [agent1; agent2; agent3; agent4];

iScenario = iScenario + 1;


end

