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

%% Initialise S, A, Q
global S
global A
S = createstatespace;
A = createactionspace;
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

%% 
oldCd = cd;
cd Scenarios
d = dir('*.m');
cd(oldCd);
fn = {d.name};
iScenario = listdlg('ListString',fn,'PromptString','Select a scenario:','SelectionMode','single');

% Run the selected scenario.
switch (iScenario)
    case 1
        crossing1;
    case 2
        crossing2;
    case 3
        headon;
    case 4
        random;
    otherwise
        error('Invalid Scenario')        
end

%% Clear variables
clear d fn oldCd iScenario

%% Run Simulation
Environment = Environment.runsimulation();

%% Save new Q
save('Q.mat','Q');
