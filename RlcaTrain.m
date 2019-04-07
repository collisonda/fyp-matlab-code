%%
% TODO: Train the model on 2, 3, and 4 agents at all different angles
% around the circle


%%
clc
close
% clear

%%
guiOn       = 0;
resetQ      = 1;
visualiseQ  = 0;

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

iScenario = 1;
runsPerScenario = 1;

count = 0;

%%
tStart = datetime('now');
nGoals = 0;
diff = zeros(1,nEpochs);
epsilon = 0.0;

% %% Main Loop
i = 1;
j = 1;
epoch = 1;
nEpochs = 4;
toGet = 3;
tic
for iStage = 1:4
    switch iStage
        case 1
            epoch = 1;
            nEpochs = 4;
            toGet = 3;
            nGoals = 0;
            Scenarios = generatescenarios();
            Scenarios = Scenarios(1:7);
            nScenarios = length(Scenarios);
            bestRun = 50*ones(1,nScenarios);
        case 2
            j = 1;
            epoch = 1;
            nEpochs = 4;
            toGet = 3;
            nGoals = 0;
            Scenarios = generatescenarios();
            Scenarios = Scenarios(8:28);
            nScenarios = length(Scenarios);
            bestRun = 50*ones(1,nScenarios);
        case 3
            j = 1;
            epoch = 1;
            nEpochs = 4;
            toGet = 3;
            nGoals = 0;
            Scenarios = generatescenarios();
            Scenarios = Scenarios(29:end);
            nScenarios = length(Scenarios);
            bestRun = 50*ones(1,nScenarios);
        case 4
            j = 1;
            epoch = 1;
            nEpochs = 2;
            toGet = 1;
            nGoals = 0;
            Scenarios = generatescenarios();
            nScenarios = length(Scenarios);
            bestRun = 50*ones(1,nScenarios);
            
        otherwise
    end
    while epoch < nEpochs
        Scenario = Scenarios{iScenario};
        %
        prevQ = Q;
        
        Environment = RlcaEnvironment(guiOn,Scenario,1,bestRun(iScenario));
        [goal, tElapsedSim] = Environment.runsimulation();
        fprintf(['Training Stage: ' num2str(iStage) '\t Epoch: ' num2str(epoch)  '\t Step: ' num2str(j) '\t Scenario: ' num2str(iScenario) '\t Result: '])
        if goal
            nGoals = nGoals + 1;
            fprintf(['GOAL \t Time: ', num2str(tElapsedSim, '%.2f') 's \t'])
            if tElapsedSim < bestRun(iScenario)
                bestRun(iScenario) = tElapsedSim;
                nGoals = nGoals + 1;
            end
        else
            nGoals = 0;
            fprintf(['COLL \t Time: ', num2str(tElapsedSim, '%.2f') 's \t'])
        end
        fprintf(['tStep: ' num2str(toc, '%.3f') '\n']);
        tic
        if nGoals == toGet
            nGoals = 0;
            iScenario = iScenario + 1;
            if iScenario > nScenarios
                iScenario = 1;
                epoch = epoch + 1;
                
            end
        end
        diff(i) = round(mean(Q(~isnan(Q)) - prevQ(~isnan(prevQ))),6);
        save('Q.mat','Q');
        save('visitCount.mat','visitCount');
        
        i = i + 1;
        j = j + 1;
        clear Environment
        close
    end
end

%% TODO: Fourth stage with random scenarios

%%
tEnd = datetime('now');
tElapsed = duration(tEnd-tStart);
tRun = tElapsed/i;
disp(tElapsed)
disp(tRun)

%%
figure
plot(1:i-1,diff)
grid on
xlabel('iRun');
ylabel('Mean Q Change');

%%
if visualiseQ
    visualiseq;
end