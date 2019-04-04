%% Housekeeping
close
clear

%% Settings to Change
guiOn = 1;
visualiseQ = 0;

%% Add folders to path
addpath('Constants')
addpath('Misc')

%% Configure graphical settings
set(0, 'DefaultFigureRenderer', 'opengl');
opengl hardware

%% Declare Globals
global epsilon
global Q
global S
global A
global visitCount

%% Assign Globals
load('Q.mat')
load('VisitCount.mat')

A = createactionspace();
S = createstatespace();
epsilon = 0;

%% Run Settings
nEpochs = 10;
nScenarios = 7;
Scenarios = generatescenarios();
iScenario = 1;
nEpochs = length(Scenarios);

%%
tStart = datetime('now');

diff = zeros(1,nEpochs);

%% Main Loop
for i = 1:nEpochs
    iScenario = mod(i-1,nScenarios+3) + 1;
    Scenario = Scenarios{iScenario};
% Scenario = generaterandomscenario;
    
    Environment = RlcaEnvironment(guiOn,Scenario,1);
    [goal, tElapsedSim] = Environment.runsimulation();
    fprintf(['Test ' num2str(i) ' of ' num2str(nEpochs) '\t Scenario: ' num2str(iScenario) '\t Result: '])
    if goal
        fprintf(['GOAL \t Time: ', num2str(tElapsedSim) 's \n'])
    else
        fprintf(['COLL \t Time: ', num2str(tElapsedSim) 's \n'])
    end
    
%     save('Q.mat','Q');
%     save('visitCount.mat','visitCount');
    clear Environment
    close
end

%%
tEnd = datetime('now');
tElapsed = duration(tEnd-tStart);
tRun = tElapsed/nEpochs;
disp(tElapsed)
disp(tRun)

%%
if visualiseQ
    visualiseq;
end