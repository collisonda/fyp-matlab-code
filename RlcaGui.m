classdef RlcaGui < handle
    %RLCAGUI: Reinforcement Learned Collision Avoidance Graphical User
    %Interface
    %   Detailed explanation goes here
    
    % TODO: Agent trails
    % TODO: Arrow showing agent heading
    
    properties
        Window
        EnvironmentPlot
        AgentPlots
        colourSettings
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
                'MarkerFaceAlpha',0.1,'MarkerEdgeColor','flat','LineWidth',2);
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
            obj.EnvironmentPlot.ColorOrder = EnvironmentConstants.COLOR_ORDER;
            obj.EnvironmentPlot.Box = 'on';
            pbaspect([1 1 1]);
            
        end
        
        function obj = updategui(obj,Agents,nAgents)
            for iAgent = 1:nAgents
                obj.AgentPlots{iAgent}.XData = Agents{iAgent}.Position.x;
                obj.AgentPlots{iAgent}.YData = Agents{iAgent}.Position.y;
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

