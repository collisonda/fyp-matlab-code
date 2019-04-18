%% Housekeeping
clc
close
clear qDiff

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
global epsilon
epsilon = -1;


%% Assign Globals
if resetQ
    load('untrainedQ.mat')
else
    load('Q.mat')
end

A = createactionspace();
S = createstatespace();

%%
iScenario = 1;
i = 1;
%%
tStart = datetime('now');

%% Main Loop

j = 1;
epoch = 1;
nEpochs = 7;
toGet = 3;
nGoals = 0;
Scenarios = generatetrainingscenarios();
Scenarios = Scenarios(1:7);
nScenarios = length(Scenarios);
successes = zeros(1,nScenarios);
tic

figure
ax1 = axes('XAxisLocation','top',...
    'YAxisLocation','left',...
    'Color',[1 1 1]);
xlabel('Epoch')
xlim([0 1])
xticks([0])
yticks([])
ax2 = axes('Position',ax1.Position,'XAxisLocation','bottom',...
    'YAxisLocation','left',...
    'Color','none');
h = animatedline('Marker','none','Color','r');
addpoints(h,0,0);
grid on
ylim([0 100])
ylabel('Success Rate %')
xlabel('Step')
drawnow
hold on
iStage = 1;
successRate = 0;
toGet = 3;
nTotalGoals = 0;
while nTotalGoals < 2*nScenarios
    Scenario = Scenarios{iScenario};
    
    prevQ = Q;
    
    Environment = RlcaEnvironment(guiOn,Scenario,1);
    [goal, tElapsedSim] = Environment.runsimulation();
    fprintf(['Epsilon: ' num2str(epsilon, '%.2f') '\t Training Stage: ' num2str(iStage) '\t Epoch: ' num2str(epoch)  '\t Step: ' num2str(j) '\t Scenario: ' num2str(iScenario) '\t Result: '])
    if goal
        successes(iScenario) = 1;
        nTotalGoals = nTotalGoals + 1;
        nGoals = nGoals + 1;
        fprintf(['GOAL \t Time: ', num2str(tElapsedSim, '%.2f') 's \t'])
    else
        successes(iScenario) = 0;
        nGoals = 0;
        nTotalGoals = 0;
        fprintf(['COLL \t Time: ', num2str(tElapsedSim, '%.2f') 's \t'])
    end
    successRate = 100*nnz(successes)/nScenarios;
    fprintf(['tStep: ' num2str(toc, '%.3f') '\t Success Rate: ' num2str(successRate, '%.2f') '%% \n']);
    
    tic

%     if nGoals == toGet
%         nGoals = 0;
%         toGet = toGet - 1;
%         if toGet < 1
%             toGet = 1;
%         end
        iScenario = iScenario + 1;
        if iScenario > nScenarios
            iScenario = 1;
            epoch = epoch + 1;
            epsilon = epsilon - 0.005;
        end
%     end
    
    qDiff(i) = round(mean(Q(~isnan(Q)) - prevQ(~isnan(prevQ))),6);
            ax1.XLim = [0 ax2.XLim(2)/nScenarios];
        ax1.XTick = [0:ax2.XLim(2)];
    
    
%     save('visitCount.mat','visitCount');
    
    i = i + 1;
    j = j + 1;

    addpoints(h,i,successRate)
    drawnow
save('Q.mat','Q');
%     close
        clear Environment
end

%     stageEnd(iStage) = j - 1;



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