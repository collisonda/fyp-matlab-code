%%
clc
clear
close

%%
set(0, 'DefaultFigureRenderer', 'painters');
opengl software

%%
Environment = RlcaEnvironment();

Environment.createAgent(0,0,30,0);
Environment.createAgent(-47,41,0,30);
Environment.createAgent(24,-28,0,30);


%%
Gui = RlcaGui(Environment);
