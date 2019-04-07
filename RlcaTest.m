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

% Scenarios = generatescenarios();
iScenario = 0;

% nScenarios = length(Scenario);
nScenarios = 6;
goals = zeros(1,nScenarios);
nEpochs = nScenarios;

%%
tStart = datetime('now');

diff = zeros(1,nEpochs);

%% Main Loop
for i = 1:nScenarios
Scenario = generaterandomscenario;
    
    Environment = RlcaEnvironment(guiOn,Scenario,1);
    [goal, tElapsedSim] = Environment.runsimulation();
    fprintf(['Test ' num2str(i) ' of ' num2str(nEpochs) '\t Scenario: ' num2str(iScenario) '\t Result: '])
    if goal
        goals(i) = 1;
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
tElapsed = seconds(duration(tEnd-tStart));
tRun = tElapsed/nEpochs;
successRate = 100*nnz(goals)/nScenarios;
fprintf(['Time Elapsed: ' num2str(tElapsed) '\t Time Per Run: ' num2str(tRun) '\t Success Rate: ' num2str(successRate) '%% \n'])

%%
if visualiseQ
    visualiseq;
end