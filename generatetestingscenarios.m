function [Scenarios] = generatetestingscenarios()

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
% x = 90*cos(pi/3);
% y = 90*sin(pi/3);
% agent1 = [0, -90, 0, 90];
% agent2 = [90, 0, -90, 0];
% Scenarios{iScenario} = [agent1; agent2];
% 
% iScenario = iScenario + 1;

%% Scenario 4
% x = 90*cos(pi/6);
% y = 90*sin(pi/6);
% agent1 = [-x, -y, x, y];
% agent2 = [x, -y, -x, y];
% Scenarios{iScenario} = [agent1; agent2];
% 
% iScenario = iScenario + 1;

%% Scenario 5
agent1 = [0, -90, 0, 90];
agent2 = [77.94, 45, -77.94, -45];
agent3 = [-77.94, 45, 77.94, -45];
Scenarios{iScenario} = [agent1; agent2; agent3];

iScenario = iScenario + 1;

%% Scenario 6
agent1 = [90, 0, -90, 0];
agent2 = [-90, 40, 90, -40];
agent3 = [-90, -40, 90, 40];
Scenarios{iScenario} = [agent1; agent2; agent3];

iScenario = iScenario + 1;

%% Scenario 7
agent1 = [1, -90, 1, 90];
agent2 = [-90, 0, 90, 0];
agent3 = [0, 90, 0, -90];
agent4 = [90, 1, -90, 1];
Scenarios{iScenario} = [agent1; agent2; agent3; agent4];

iScenario = iScenario + 1;

%% Scenario 8
agent1 = [90, 20, -90, 20];
agent2 = [90, -20, -90, -20];
agent3 = [-90, 20, 90, 20];
agent4 = [-90, -20, 90, -20];
Scenarios{iScenario} = [agent1; agent2; agent3; agent4];

iScenario = iScenario + 1;


end
