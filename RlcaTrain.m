%%
close
clear

%%
guiOn = 1;
resetQ = 0;
visualiseQ = 0;

%% Add folders to path
addpath('Constants')
addpath('Misc')
addpath('Scenarios')

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
if resetQ
    load('untrainedQ.mat')
    load('blankVisitCount.mat')
else
    load('Q.mat')
    load('VisitCount.mat')
end

A = createactionspace();
S = createstatespace();

%%
nRuns = 7*255;
nScenarios = 7;
Scenarios = generatescenarios(nScenarios);
iScenario = 1;
runsPerScenario = 128;

%%
epsilonArray = zeros(1,nRuns);
epsilonArray(nScenarios*runsPerScenario:(nScenarios*runsPerScenario+nScenarios*runsPerScenario/2)) = 0.1;

%%
tStart = datetime('now');

diff = zeros(1,nRuns);

%% Main Loop
for i = 1:nRuns
    epsilon = epsilonArray(i);

    Scenario = Scenarios{iScenario};
    fprintf(['Epsilon: ' num2str(epsilon) '\t Training Step ' num2str(i) ' of ' num2str(nRuns) '\t Scenario: ' num2str(iScenario) '\t Result: '])
    clear Environment
    prevQ = Q;
    
    Environment = RlcaEnvironment(guiOn,Scenario);
    [goal, tElapsedSim] = Environment.runsimulation();
    if goal
        fprintf(['GOAL \t Time: ', num2str(tElapsedSim) 's \n'])
    else
        fprintf(['COLL \t Time: ', num2str(tElapsedSim) 's \n'])
    end
    
        
    save('Q.mat','Q');
    save('visitCount.mat','visitCount');
    
    
    diff(i) = round(mean(Q(~isnan(Q)) - prevQ(~isnan(prevQ))),6);
    
    if mod(i,runsPerScenario) == 0
        iScenario = mod(i/runsPerScenario,nScenarios) + 1;
        if iScenario == 1 && runsPerScenario > 1
            runsPerScenario = runsPerScenario/2;
        end
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
figure
plot(1:nRuns,diff)
grid on
xlabel('iRun');
ylabel('Mean Q Change');

%%
if visualiseQ
    visualiseq;
end