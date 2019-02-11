classdef RlcaGui < handle
    %RLCAGUI: Reinforcement Learned Collision Avoidance Graphical User
    %Interface
    %   Detailed explanation goes here
    
    % TODO: Arrow showing agent heading
    
    properties
        Window
        EnvironmentPlot
        AgentPlots
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
            theta = Agent.heading;
            obj.AgentHeadings{iAgent} = plot([x x+3*cos(theta)],[y y+3*sin(theta)],'Color',Agent.color,'LineWidth',1.5);
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
            pbaspect([1 1 1]);
            obj.time = annotation('textbox',[0.700520833333333 ...
                0.924826389328679 0.122656248544808 0.0369791662268766],...
                'String','Time: 0.0s','FontSize',12,'FontWeight','bold',...
                'BackgroundColor',[1 1 1],'VerticalAlignment','middle');
            
        end
        
        function obj = updategui(obj,Agents,nAgents,time)
            for iAgent = 1:nAgents
                obj.time.String = ['Time: ' num2str(time)];
                x = Agents{iAgent}.Position.x;
                y = Agents{iAgent}.Position.y;
                obj.AgentPlots{iAgent}.XData = x;
                obj.AgentPlots{iAgent}.YData = y;
                obj.AgentTrails{iAgent}.addpoints(x,y);
                theta = Agents{iAgent}.heading;
                obj.AgentHeadings{iAgent}.XData = [x x+3*cos(theta)];
                obj.AgentHeadings{iAgent}.YData = [y y+3*sin(theta)];
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

