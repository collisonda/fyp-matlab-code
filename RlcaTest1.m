RlcaSetup;

figure

agent1 = Agent(0,0,10,10);
agent1.Velocity.actual = 1;
agent1.heading = 10;

agent2 = Agent(-15,15,10,10);
agent2.Velocity.actual = 2;
agent2.heading = -45;

hold on
agent1Plot = plot(agent1.Position.x,agent1.Position.y,'^');
agent1RadiusPlot = plot(agent1.Position.x,agent1.Position.y,'o','MarkerSize',agent1.radius*10);
agent2Plot = plot(agent2.Position.x,agent2.Position.y,'^');
agent2RadiusPlot = plot(agent2.Position.x,agent2.Position.y,'o','MarkerSize',agent2.radius*10);

xlim([-40 40])
ylim([-40 40])
pbaspect([1 1 1])
box on

for i = 1:200
   agent1 = agent1.timeStep(0.1);
   agent1Plot.XData = agent1.Position.x;
   agent1Plot.YData = agent1.Position.y;
   agent1RadiusPlot.XData = agent1.Position.x;
   agent1RadiusPlot.YData = agent1.Position.y;
   
   agent2 = agent2.timeStep(0.1);
   agent2Plot.XData = agent2.Position.x;
   agent2Plot.YData = agent2.Position.y;
   agent2RadiusPlot.XData = agent2.Position.x;
   agent2RadiusPlot.YData = agent2.Position.y;
   
   drawnow
end