%%
clc
clear
close

%%
opengl software

%%
Environment = RlcaEnvironment();
Environment.createAgent(0,0,30,30);
Environment.createAgent(80,-30,0,-60);

%%
Gui = RlcaGui(Environment);
