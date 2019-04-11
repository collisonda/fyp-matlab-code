%TODO: Redo Q calculation to match how its supposed to work for Q learning




%% Housekeeping
clc
close

%% Settings to Change
guiOn       = 0; % Toggles GUI.
resetQ      = 1; % Toggles resetting Q before training.
visualiseQ  = 0; % Toggles the visualisation of Q after training.

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
i = 1;
%% 
tStart = datetime('now');

%% Main Loop
for iStage = 2:4
    switch iStage
        case 1
            j = 1;
            epoch = 1;
            nEpochs = 4;
            toGet = 3;
            nGoals = 0;
            Scenarios = generatetrainingscenarios();
            Scenarios = Scenarios(1:7);
            nScenarios = length(Scenarios);
            tic
        case 2
            j = 1;
            epoch = 1;
            nEpochs = 4;
            toGet = 3;
            nGoals = 0;
            Scenarios = generatetrainingscenarios();
            Scenarios = Scenarios(8:28);
            nScenarios = length(Scenarios);
            tic
        case 3
            j = 1;
            epoch = 1;
            nEpochs = 4;
            toGet = 3;
            nGoals = 0;
            Scenarios = generatetrainingscenarios();
            Scenarios = Scenarios(29:end);
            nScenarios = length(Scenarios);
            tic
        case 4
            j = 1;
            epoch = 1;
            nEpochs = 2;
            toGet = 1;
            nGoals = 0;
            Scenarios = generatetrainingscenarios();
            nScenarios = length(Scenarios);
            tic            
        otherwise
    end
    
    while epoch < nEpochs
        Scenario = Scenarios{iScenario};
        
        prevQ = Q;
        
        Environment = RlcaEnvironment(guiOn,Scenario,1);
        [goal, tElapsedSim] = Environment.runsimulation();
        fprintf(['Training Stage: ' num2str(iStage) '\t Epoch: ' num2str(epoch)  '\t Step: ' num2str(j) '\t Scenario: ' num2str(iScenario) '\t Result: '])
        if goal
            nGoals = nGoals + 1;
            fprintf(['GOAL \t Time: ', num2str(tElapsedSim, '%.2f') 's \t'])
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
        
        qDiff(i) = round(mean(Q(~isnan(Q)) - prevQ(~isnan(prevQ))),6);
        
        save('Q.mat','Q');
        save('visitCount.mat','visitCount');
        
        i = i + 1;
        j = j + 1;
        clear Environment
        close
    end
    
    stageEnd(iStage) = j - 1;
    
end

% %% Random Stage
% iStage = 5;
% nRuns = 500;
% j = 1;
% results = zeros(1,50);
% nGoals = 0;
% Scenario = generaterandomscenario;
% while nGoals < 20
%     
%     %
%     prevQ = Q;
%     
%     Environment = RlcaEnvironment(guiOn,Scenario,1);
%     [goal, tElapsedSim] = Environment.runsimulation();
%     fprintf(['Training Stage: ' num2str(iStage) '\t Step: ' num2str(j) '\t Scenario: RAND' '\t Result: '])
%     if goal
%         Scenario = generaterandomscenario;
%         nGoals = nGoals + 1;
%         fprintf(['GOAL \t Time: ', num2str(tElapsedSim, '%.2f') 's \t'])
%         results = [1 results(1:19)];
%     else
%         nGoals = 0;
%         fprintf(['COLL \t Time: ', num2str(tElapsedSim, '%.2f') 's \t'])
%         results = [0 results(1:19)];
%     end
%     successRate = 100*nnz(results)/50;
%     fprintf(['tStep: ' num2str(toc, '%.3f') '\t Success Rate: ' num2str(successRate) '%% \n']);
%     tic
%     qDiff(i) = round(mean(Q(~isnan(Q)) - prevQ(~isnan(prevQ))),6);
%     save('Q.mat','Q');
%     save('visitCount.mat','visitCount');
%     
%     i = i + 1;
%     j = j + 1;
%     clear Environment
%     close
% end


%% Display Stats
tEnd = datetime('now');
tElapsed = minutes(duration(tEnd-tStart));
tRun = tElapsed/i;
fprintf(['Time Elapsed: ' num2str(tElapsed) '\t Time Per Run: ' num2str(tRun) '\n'])

%% Plot Q over time
figure
plot(1:i-1,qDiff,'-','Color',[0.5 0.5 0.5])
hold on
plot(1:i-1,smooth(qDiff,200),'-r','LineWidth',1.5)
grid on
xlabel('Step');
ylabel('Q Change');

%% Visualise Q
if visualiseQ
    visualiseq;
end