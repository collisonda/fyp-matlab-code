classdef RlcaGui < handle
    %RLCAGUI: Reinforcement Learned Collision Avoidance Graphical User
    %Interface
    %   Detailed explanation goes here
    
    % TODO: Arrow showing agent heading
    
    %% RlcaGui - Properties
    properties
        Window
        EnvironmentPlot
        AgentPlots
        AgentNumbers
        AgentTrails
        AgentGoals
        AgentHeadings
        AgentVision
        colourSettings
        time
    end
    
    %% RlcaGui - Public Methods
    methods (Access = public)
        
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
            x = Agent.position(1);
            y = Agent.position(2);
            r = Agent.radius;
            
            iAgent = Agent.iAgent;
            hold on
            ax = gca;
            oldUnits = get(ax, 'Units');
            set(ax, 'Units', 'points');
            posPoints = get(ax, 'Position');
            set(ax, 'Units', oldUnits);
            narrower_part_points = min(posPoints(3:4));
            obj.AgentPlots{iAgent} = scatter(x,y,(narrower_part_points^2)/...
                (100/r)^2,'filled','MarkerFaceAlpha',0.1,'MarkerEdgeColor',...
                Agent.color,'MarkerFaceColor',Agent.color,'LineWidth',2);
            obj.AgentTrails{iAgent} = animatedline(x,y,'Color',Agent.color,...
                'LineStyle','--','LineWidth',1.5);
            obj.AgentGoals{iAgent} = scatter(Agent.goal(1),Agent.goal(2),100,...
                'x','MarkerEdgeColor',Agent.color,'LineWidth',1.5);
            obj.AgentNumbers{iAgent} = text(x,y+4,num2str(iAgent),'Color',...
                Agent.color,'HorizontalAlignment','center','FontWeight','bold');
            obj.AgentHeadings{iAgent} = plot([x x+2*cos(Agent.heading)],...
                [y y+2*sin(Agent.heading)],'Color',Agent.color,'LineWidth',1.5);
            [nx, ny] = createarc(Agent.heading + ...
                AgentConstants.VISION_ANGLE,Agent.heading - ...
                AgentConstants.VISION_ANGLE,Agent.position(1),...
                Agent.position(2),AgentConstants.NEIGHBOURHOOD_RADIUS);
            obj.AgentVision{iAgent} = fill(nx,ny,Agent.color,'EdgeColor',...
                'none','FaceAlpha',0.1);
            hold off
        end
        
        function obj = updategui(obj,Agents,nAgents)
            for iAgent = 1:nAgents
                x = Agents{iAgent}.position(1);
                y = Agents{iAgent}.position(2);
                obj.AgentPlots{iAgent}.XData = x;
                obj.AgentPlots{iAgent}.YData = y;
                obj.AgentNumbers{iAgent}.Position = [x,y+4,0];
                obj.AgentTrails{iAgent}.addpoints(x,y);
                theta = Agents{iAgent}.heading;
                obj.AgentHeadings{iAgent}.XData = [x x+2*cos(theta)];
                obj.AgentHeadings{iAgent}.YData = [y y+2*sin(theta)];
                obj.AgentGoals{iAgent}.XData = Agents{iAgent}.goal(1);
                obj.AgentGoals{iAgent}.YData = Agents{iAgent}.goal(2);
                [nx, ny] = createarc(Agents{iAgent}.heading + ...
                AgentConstants.VISION_ANGLE,Agents{iAgent}.heading - ...
                AgentConstants.VISION_ANGLE,Agents{iAgent}.position(1),...
                Agents{iAgent}.position(2),AgentConstants.NEIGHBOURHOOD_RADIUS);
                obj.AgentVision{iAgent}.XData = nx;
                obj.AgentVision{iAgent}.YData = ny;
            end
            drawnow
        end
        
    end
    
    %% RlcaGui - Private Methods
    methods (Access = private)
        
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
            pbaspect([1 1 1])
        end
    end
    
    %% RlcaGui - Static Methods
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

