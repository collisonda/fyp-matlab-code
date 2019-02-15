
%% Housekeeping
clc % Clear command line
clear % Clear workspace
close % Close any figures

%%
addpath('Constants')
addpath('Misc')

%% Configure graphical settings
set(0, 'DefaultFigureRenderer', 'painters');
opengl software

%% Initialise Environment
Environment = RlcaEnvironment();

%% Create Agents
% TODO: Have a user option to select the scenario (i.e. 2 agents crossing
% paths, or 4 agents etc.)
Environment.createagent(-55,-50,50,55);
Environment.createagent(55,50,-50,-55);

% for i = 1:9
%     Environment.createagent(100*(rand-rand),100*(rand-rand),100*(rand-rand),100*(rand-rand));
% end
% clear i

%% Run Simulation
Environment = Environment.run();