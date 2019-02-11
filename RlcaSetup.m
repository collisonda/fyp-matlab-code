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
Environment.createagent(-30,-30,30,30);
Environment.createagent(30,30,-30,-30);

%%
Environment = Environment.run();

