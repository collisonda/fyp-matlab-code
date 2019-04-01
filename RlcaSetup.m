function [] = RlcaSetup(Scenario,guiOn)

tOptimal = 30;
Environment = RlcaEnvironment(guiOn,eventsOn,tOptimal);

agent1 = Scenario(1,:);
agent2 = Scenario(2,:);
Environment.createagent(agent1(1),agent1(2),-agent1(1),-agent1(2));
Environment.createagent(agent2(1),agent2(2),-agent2(1),-agent2(2));


%% Run Simulation
Environment = Environment.runsimulation();
end
