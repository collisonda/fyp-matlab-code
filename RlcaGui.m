classdef RlcaGui < handle
    %RLCAGUI: Reinforcement Learned Collision Avoidance Graphical User
    %Interface
    %   Detailed explanation goes here
    
    % TODO: Arrow showing agent heading
    
    properties
        Window
        EnvironmentPlot
        AgentPlots
        AgentNumbers
        AgentTrails
        AgentGoals
        AgentHeadings
        colourSettings
        time
    end
    
    methods
        function obj = RlcaGui()
            %RLCAGUI Construct an instance of this class
            %   Detailed explanation goes here
            
            windowSettings = obj.getwindowsettings();
            obj.Window = figure('Name','RLCA GUI','Position',windowSettings,...
                'GraphicsSmoothing','on','Resize','off');
            obj.setupenvironmentplot();
        end
        
        function obj = generateagentgraphic(obj,Agent)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            x = Agent.Position.x;
            y = Agent.Position.y;
            r = Agent.radius;
            
            iAgent = Agent.iAgent;
            hold on
            obj.AgentPlots{iAgent} = scatter(x,y,r*200,'filled',...
                'MarkerFaceAlpha',0.1,'MarkerEdgeColor',Agent.color,...
                'MarkerFaceColor',Agent.color,'LineWidth',2);
            obj.AgentTrails{iAgent} = animatedline(x,y,'Color',Agent.color,...
                'LineStyle','--','LineWidth',1.5);
            obj.AgentGoals{iAgent} = scatter(Agent.Goal.x,Agent.Goal.y,100,...
                'x','MarkerEdgeColor',Agent.color,'LineWidth',1.5);
            obj.AgentNumbers{iAgent} = text(x,y+4,num2str(iAgent),'Color',Agent.color,'HorizontalAlignment','center','FontWeight','bold');
            theta = Agent.heading;
            obj.AgentHeadings{iAgent} = plot([x x+2*cos(theta)],[y y+2*sin(theta)],'Color',Agent.color,'LineWidth',1.5);
            hold off
        end
        
        function obj = setupenvironmentplot(obj)
            obj.EnvironmentPlot = axes(obj.Window);
            obj.EnvironmentPlot.XLim = EnvironmentConstants.X_BOUNDARY;
            obj.EnvironmentPlot.YLim = EnvironmentConstants.Y_BOUNDARY;
            obj.EnvironmentPlot.GridColor = [0.05 0.05 0.05];
            obj.EnvironmentPlot.XGrid = 'on';
            obj.EnvironmentPlot.YGrid = 'on';
            obj.EnvironmentPlot.XLabel.String = 'X Position';
            obj.EnvironmentPlot.XLabel.FontWeight = 'bold';
            obj.EnvironmentPlot.YLabel.String = 'Y Position';
            obj.EnvironmentPlot.YLabel.FontWeight = 'bold';
            obj.EnvironmentPlot.Box = 'on';
        end
        
        function obj = updategui(obj,Agents,nAgents,time)
            for iAgent = 1:nAgents
                x = Agents{iAgent}.Position.x;
                y = Agents{iAgent}.Position.y;
                obj.AgentPlots{iAgent}.XData = x;
                obj.AgentPlots{iAgent}.YData = y;
                obj.AgentNumbers{iAgent}.Position = [x,y+4,0];
                obj.AgentTrails{iAgent}.addpoints(x,y);
                theta = Agents{iAgent}.heading;
                obj.AgentHeadings{iAgent}.XData = [x x+2*cos(theta)];
                obj.AgentHeadings{iAgent}.YData = [y y+2*sin(theta)];
            end
            drawnow
        end
        
    end
    
    methods (Static)
        function windowSettings = getwindowsettings()
            screenSize = get(0,'ScreenSize');
            xPos = 0.2*screenSize(3);
            yPos = 0.1*screenSize(4);
            xSize = 0.6*screenSize(3);
            ySize = 0.8*screenSize(4);
            windowSettings = [xPos,yPos,xSize,ySize];
        end
        
    end
end

