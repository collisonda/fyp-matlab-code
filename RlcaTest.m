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
global epsilon
epsilon = 0;

%% Assign Globals
load('Q.mat')

A = createactionspace();
S = createstatespace();

%% Run Settings
nEpochs = 10;

Scenarios = generatetestingscenarios();
Scenarios = Scenarios(4);
iScenario = 1;

nScenarios = length(Scenarios);
% nScenarios = 100;
goals = zeros(1,nScenarios);
nEpochs = nScenarios;

%%
tStart = datetime('now');
tElapsedSimTotal = 0;
%% Main Loop
for i = 1:nScenarios
% Scenario = generaterandomscenario;
Scenario = Scenarios{iScenario};
    
    Environment = RlcaEnvironment(guiOn,Scenario,0);
    [goal, tElapsedSim] = Environment.runsimulation();
    fprintf(['Test ' num2str(i) ' of ' num2str(nEpochs) '\t Scenario: ' num2str(iScenario) '\t Result: '])
    if goal
        goals(i) = 1;
        fprintf(['GOAL \t Time: ', num2str(tElapsedSim) 's \n'])
    else
        fprintf(['COLL \t Time: ', num2str(tElapsedSim) 's \n'])
    end
    tElapsedSimTotal = tElapsedSimTotal + tElapsedSim;
    iScenario = iScenario + 1;
%     save('Q.mat','Q');
%     save('visitCount.mat','visitCount');
    clear Environment
%     pause
    close

end

%%
tEnd = datetime('now');
tElapsed = seconds(duration(tEnd-tStart));
tRun = tElapsed/nScenarios;
successRate = 100*nnz(goals)/nScenarios;
fprintf(['Simulation Time Elapsed: ' num2str(tElapsedSimTotal) '\t Time Per Run: ' num2str(tRun) '\t Success Rate: ' num2str(successRate) '%% \n'])

%%
if visualiseQ
    visualiseq;
end