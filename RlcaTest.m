%%
close
clear

%%
guiOn = 1;
visualiseQ = 0;

%% Add folders to path
addpath('Constants')
addpath('Misc')

%% Configure graphical settings
set(0, 'DefaultFigureRenderer', 'opengl');
opengl hardware

%%
global epsilon
global Q
global S
global A
global visitCount

%%
load('Q.mat')
load('VisitCount.mat')

A = createactionspace();
S = createstatespace();

%%
nRuns = 7;
nScenarios = 7;
Scenarios = generatescenarios(nScenarios);
iScenario = 1;

epsilon = 0;

%%
tStart = datetime('now');

diff = zeros(1,nRuns);

%% Main Loop
for i = 1:nRuns
    iScenario = mod(i-1,nScenarios) + 1;
    Scenario = Scenarios{iScenario};
    
    
    Environment = RlcaEnvironment(guiOn,Scenario,0);
    [goal, tElapsedSim] = Environment.runsimulation();
    fprintf(['Test ' num2str(i) ' of ' num2str(nRuns) '\t Scenario: ' num2str(iScenario) '\t Result: '])
    if goal
        fprintf(['GOAL \t Time: ', num2str(tElapsedSim) 's \n'])
    else
        fprintf(['COLL \t Time: ', num2str(tElapsedSim) 's \n'])
    end
    clear Environment
    close
end

%%
tEnd = datetime('now');
tElapsed = duration(tEnd-tStart);
tRun = tElapsed/nRuns;
disp(tElapsed)
disp(tRun)

%%
if visualiseQ
    visualiseq;
end