%% Housekeeping
clc % Clear command line
clear % Clear workspace
close % Close any figures

%% Add folders to path
addpath('Constants')
addpath('Misc')

%% Configure graphical settings
set(0, 'DefaultFigureRenderer', 'opengl');
opengl hardware

%% Initialise Environment
Environment = RlcaEnvironment();

%% Initialise Q
load('Q.mat');
global Q

%% Create Agents
% TODO: Have a user option to select the scenario (i.e. 2 agents crossing
% paths, or 4 agents etc.)
oldCd = cd;
cd Scenarios
d = dir('*.m');
cd(oldCd);
fn = {d.name};
[idx,tf] = listdlg('ListString',fn,'PromptString','Select a scenario:','SelectionMode','single');

switch (idx)
    case 1
        crossing1;
    case 2
        crossing2;
    case 3
        headon;
    otherwise
        
end
% Environment.createagent(-50,-50,50,50);
% Environment.createagent(50,50,-50,-50);
% Environment.createagent(-50,50,50,-50);
% Environment.createagent(50,-50,-50,50);
% Environment.createagent(0,70.7107,0,-70.7107);
% Environment.createagent(0,-70.7107,0,70.7107);
% Environment.createagent(-70.7107,0,70.7107,0);
% Environment.createagent(70.7107,0,-70.7107,0);

% for i = 1:4
%     Environment.createagent(100*(rand-rand),100*(rand-rand),100*(rand-rand),100*(rand-rand));
% end
% clear i

%% Run Simulation
Environment = Environment.run();

%% Save new Q
save('Q.mat','Q');
