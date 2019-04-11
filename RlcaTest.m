%% Housekeeping
close
clear

%% Settings to Change
guiOn       = 1; % Toggles GUI.
visualiseQ  = 0; % Toggles the visualisation of Q after testing.

%% Add folders to path
addpath('Constants')
addpath('Misc')

%% Configure graphical settings
set(0, 'DefaultFigureRenderer', 'opengl');
opengl hardware

%% Declare Globals
global Q
global S
global A
global visitCount

%% Assign Globals
load('Q.mat')
load('visitCount.mat')

A = createactionspace();
S = createstatespace();

%% Run Settings
nEpochs = 10;

Scenarios = generatetestingscenarios();
iScenario = 1;

nScenarios = length(Scenarios);
goals = zeros(1,nScenarios);
nEpochs = nScenarios;

%%
tStart = datetime('now');

%% Main Loop
for i = 1:nScenarios
% Scenario = generaterandomscenario;
Scenario = Scenarios{iScenario};
    
    Environment = RlcaEnvironment(guiOn,Scenario,1);
    [goal, tElapsedSim] = Environment.runsimulation();
    fprintf(['Test ' num2str(i) ' of ' num2str(nEpochs) '\t Scenario: ' num2str(iScenario) '\t Result: '])
    if goal
        goals(i) = 1;
        fprintf(['GOAL \t Time: ', num2str(tElapsedSim) 's \n'])
    else
        fprintf(['COLL \t Time: ', num2str(tElapsedSim) 's \n'])
    end
    
    iScenario = iScenario + 1;
%     save('Q.mat','Q');
%     save('visitCount.mat','visitCount');
    clear Environment
    close
end

%%
tEnd = datetime('now');
tElapsed = seconds(duration(tEnd-tStart));
tRun = tElapsed/nEpochs;
successRate = 100*nnz(goals)/nScenarios;
fprintf(['Time Elapsed: ' num2str(tElapsed) '\t Time Per Run: ' num2str(tRun) '\t Success Rate: ' num2str(successRate) '%% \n'])

%%
if visualiseQ
    visualiseq;
end