%% Housekeeping
clc     % Clear command line
clear   % Clear workspace
close   % Close any figures

%% Add folders to path
addpath('Constants')
addpath('Misc')
addpath('Scenarios')

%% Configure graphical settings
set(0, 'DefaultFigureRenderer', 'opengl');
opengl hardware

%% Initialise Environment
Environment = RlcaEnvironment();

%% Initialise Q
load('Q.mat');
global Q
% TODO: Setup Q so that for each state, every action has a Q value
% associated with it.
% A state is the neighbour's position relative to the position of the
% agent.
% The action set is every possible combination of X and Y velocities.
% Therefore, for each neighbour location, there's an optimal velocity that
% will be found through the Q values.
% The Q values are calculated based on three criterions:
% >> How close the chosen velocity is to the current velocity.
% >> The change in distance to goal.
% >> Whether or not the velocity choice has resulted in a collision.
% >> Successive velocity choices need to impact past Q values so that even
%    if that particular timestep/velocity choice did not cause a collision,
%    it contributed to one. i.e. if the agent chose a series such as V1, V2
%    V3, and V3 resulted in a collision, past choices V2 and V1 should have
%    their Q values reduced too, V2 more than V1 as it was closer in time
%    to the collision.

%% Create Agents
% TODO: Have a user option to select the scenario (i.e. 2 agents crossing
% paths, or 4 agents etc.)
oldCd = cd;
cd Scenarios
d = dir('*.m');
cd(oldCd);
fn = {d.name};
idx = listdlg('ListString',fn,'PromptString','Select a scenario:','SelectionMode','single');

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

clear d fn oldCd tf
%% Run Simulation
Environment = Environment.runsimulation();

%% Save new Q
save('Q.mat','Q');
