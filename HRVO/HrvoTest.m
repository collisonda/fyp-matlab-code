%% Housekeeping
close
clear

%% Settings to Change
guiOn       = 1; % Toggles GUI.

%% Add folders to path
addpath('Constants')
addpath('Misc')
addpath('RVO')

%% Configure graphical settings
set(0, 'DefaultFigureRenderer', 'opengl');
opengl hardware

Scenarios = generatetestingscenarios();
iScenario = 1;

nScenarios = length(Scenarios);
goals = zeros(1,nScenarios);

%%
tStart = datetime('now');
tElapsedSimTotal = 0;
%% Main Loop
for i = 1:nScenarios
% Scenario = generaterandomscenario;
Scenario = Scenarios{iScenario};
    
    Environment = HrvoEnvironment(guiOn,Scenario);
    [goal, tElapsedSim] = Environment.runsimulation();
    fprintf(['Test ' num2str(i) ' of ' num2str(nScenarios) '\t Scenario: ' num2str(iScenario) '\t Result: '])
    if goal
        goals(i) = 1;
        fprintf(['GOAL \t Time: ', num2str(tElapsedSim) 's \n'])
    else
        fprintf(['COLL \t Time: ', num2str(tElapsedSim) 's \n'])
    end
    tElapsedSimTotal = tElapsedSimTotal + tElapsedSim;
    iScenario = iScenario + 1;
    clear Environment
    close
end

%%
tEnd = datetime('now');
tElapsed = seconds(duration(tEnd-tStart));
tRun = tElapsed/nScenarios;
successRate = 100*nnz(goals)/nScenarios;
fprintf(['Simulation Time Elapsed: ' num2str(tElapsedSimTotal) '\t Time Per Run: ' num2str(tRun) '\t Success Rate: ' num2str(successRate) '%% \n'])