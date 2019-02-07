%%
clc
clear
close

%%
opengl software

%%
Environment = RlcaEnvironment();
Environment.createAgent(0,0,30,30);

%%
Gui = RlcaGui(Environment);
