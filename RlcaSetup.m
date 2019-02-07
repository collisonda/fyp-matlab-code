%%
clc
clear
close

%%
opengl software

%%
Environment = RlcaEnvironment();

Environment.createAgent(63,0,30,0);
Environment.createAgent(-47,41,0,30);
Environment.createAgent(24,-28,0,30);


%%
Gui = RlcaGui(Environment);
