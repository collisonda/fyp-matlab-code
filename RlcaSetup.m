%%
clc
clear
close

%%
set(0, 'DefaultFigureRenderer', 'painters');
opengl software

%%
Gui = RlcaGui();

%%
Environment = RlcaEnvironment(Gui);

%%
% TODO: Have a user option to select the scenario (i.e. 2 agents crossing
% paths, or 4 agents etc.)
Environment.createagent(-55,-50,50,55);
Environment.createagent(55,50,-50,-55);

% for i = 1:9
%     Environment.createagent(94*(rand-rand),94*(rand-rand),94*(rand-rand),94*(rand-rand));
% end

%%
Environment = Environment.run();

