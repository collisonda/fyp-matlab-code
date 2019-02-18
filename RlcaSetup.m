
%% Housekeeping
clc % Clear command line
clear % Clear workspace
close % Close any figures

%%
addpath('Constants')
addpath('Misc')

%% Configure graphical settings
set(0, 'DefaultFigureRenderer', 'opengl');
opengl hardware

%% Initialise Environment
Environment = RlcaEnvironment();

%% Create Agents
% TODO: Have a user option to select the scenario (i.e. 2 agents crossing
% paths, or 4 agents etc.)
Environment.createagent(-50,-50,50,50);
Environment.createagent(50,50,-50,-50);
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