classdef RlcaGui < handle
    %RLCAGUI: Reinforcement Learned Collision Avoidance Graphical User
    %Interface
    %   Detailed explanation goes here
    
    properties
        Window
        EnvironmentPlot
        agentGraphics
        colourSettings
    end
    
    methods
        function obj = RlcaGui(Environment)
            %RLCAGUI Construct an instance of this class
            %   Detailed explanation goes here
            windowSettings = obj.getWindowSettings();
            obj.Window = figure('Name','RLCA GUI','Position',windowSettings);
            obj.setupEnvironmentPlot();
        end
        
        function obj = generateAgentGraphic(obj,a)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
        end
        
        function obj = setupEnvironmentPlot(obj)
            obj.EnvironmentPlot = axes(obj.Window);
            obj.EnvironmentPlot.XLim = EnvironmentConstants.xBoundary;
            obj.EnvironmentPlot.YLim = EnvironmentConstants.yBoundary;
            obj.EnvironmentPlot.GridColor = [0.05 0.05 0.05];
            obj.EnvironmentPlot.XGrid = 'on';
            obj.EnvironmentPlot.YGrid = 'on';
            obj.EnvironmentPlot.XLabel.String = 'X Position';
            obj.EnvironmentPlot.XLabel.FontWeight = 'bold';
            obj.EnvironmentPlot.YLabel.String = 'Y Position';
            obj.EnvironmentPlot.YLabel.FontWeight = 'bold';
            obj.EnvironmentPlot.ColorOrder = [0 0.1 1
                                              1 0 0.2
                                              0.4 0.9 0
                                              1 0.4 0
                                              0.6 0 1
                                              1 0.3 1
                                              1 0.8 0
                                              0.35 0.95 0.75
                                              0 0.9 1
                                              ];
            obj.EnvironmentPlot.Box = 'on';
            
        end
        
    end
    
    methods (Static)
        function windowSettings = getWindowSettings()
            screenSize = get(0,'ScreenSize');
            xPos = 0.2*screenSize(3);
            yPos = 0.1*screenSize(4);
            xSize = 0.6*screenSize(3);
            ySize = 0.8*screenSize(4);
            windowSettings = [xPos,yPos,xSize,ySize];
        end
        
    end
end

