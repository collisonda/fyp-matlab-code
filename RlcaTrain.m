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
nEpochs = 4;
nScenarios = 7;
Scenarios = generatescenarios(nScenarios);
iScenario = 1;
runsPerScenario = 32;
bestRun = 50*ones(1,nScenarios+3);
Scenario = generaterandomscenario;
Scenario = [-80.6432, 5.2012; -1.5845, 28.5374; -92.5098, 58.8893; 49.2779, 62.4745];
size(Scenario,1)
count = 0;

%%
tStart = datetime('now');
nGoals = 0;
diff = zeros(1,nEpochs);
epsilon = 0.0;

%% Main Loop
i = 1;
epoch = 1;
toGet = 3;
while epoch < nEpochs
%     Scenario = Scenarios{iScenario};
%     
    prevQ = Q;
    
    Environment = RlcaEnvironment(guiOn,Scenario,1,bestRun(iScenario));
    [goal, tElapsedSim] = Environment.runsimulation();
    fprintf(['Epoch: ' num2str(epoch) '\t Training Stage 1, Step ' num2str(i) '\t Scenario: ' num2str(iScenario) '\t Result: '])
    if goal
        nGoals = nGoals + 1;
        fprintf(['GOAL \t Time: ', num2str(tElapsedSim) 's \n'])
        if tElapsedSim < bestRun(iScenario)
            bestRun(iScenario) = tElapsedSim;
        end
    else
        nGoals = 0;
        fprintf(['COLL \t Time: ', num2str(tElapsedSim) 's \n'])
    end
    %     iScenario = iScenario + 1;
    %     if iScenario > nScenarios
    %         iScenario = 1;
    %         run = run + 1;
    %     end
    
%     if nGoals == toGet
%         iScenario = iScenario + 1;
%         
%         nGoals =0;
%         if iScenario > nScenarios + 3
%             iScenario = 1;
%             epoch = epoch + 1;
%             toGet = toGet - 1;
%             if toGet < 1
%                 toGet = 1;
%             end
%         end
%     end

if nGoals == toGet
   Scenario = generaterandomscenario;
   epoch = epoch + 1;
end

    diff(i) = round(mean(Q(~isnan(Q)) - prevQ(~isnan(prevQ))),6);
    save('Q.mat','Q');
    save('visitCount.mat','visitCount');
    
    i = i + 1;
    clear Environment
    close
end

%%
% epsilon = 0.5;
% iScenario = 1;
% for i = 1:nRuns
%     
%     Scenario = Scenarios{iScenario};
%     
%     prevQ = Q;
%     
%     Environment = RlcaEnvironment(guiOn,Scenario,2,bestRun(iScenario));
%     [goal, tElapsedSim] = Environment.runsimulation();
%     fprintf(['Epsilon: ' num2str(epsilon) '\t Training Stage 2, Step ' num2str(i) ' of ' num2str(nRuns) '\t Scenario: ' num2str(iScenario) '\t Result: '])
%     if goal
%         nGoals = nGoals + 1;
%         fprintf(['GOAL \t Time: ', num2str(tElapsedSim) 's \n'])
%         if tElapsedSim < bestRun(iScenario)
%             bestRun(iScenario) = tElapsedSim;
%             disp('Found new best run')
%         end
%     else
%         nGoals = 0;
%         fprintf(['COLL \t Time: ', num2str(tElapsedSim) 's \n'])
%     end
%     
%     
%     save('Q.mat','Q');
%     save('visitCount.mat','visitCount');
%     
%     
%     diff(i) = round(mean(Q(~isnan(Q)) - prevQ(~isnan(prevQ))),6);
%     
%     if mod(i,runsPerScenario) == 0
%         iScenario = mod(i/runsPerScenario,nScenarios) + 1;
%         if iScenario == 1 && runsPerScenario > 1
%             runsPerScenario = runsPerScenario/2;
%         end
%     end
%     
%     clear Environment
%     close
% end

%%
tEnd = datetime('now');
tElapsed = duration(tEnd-tStart);
tRun = tElapsed/nEpochs;
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