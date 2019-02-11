classdef RlcaEnvironment < handle
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Agents = cell(0);
        nAgents = 0;
        time = 0;
        Gui
    end
    
    methods
        function obj = RlcaEnvironment(Gui)
            obj.Gui = Gui;
            
        end
        
        function obj = run(obj)
            for t = EnvironmentConstants.START_TIME:...
                    EnvironmentConstants.TIME_STEP:...
                    EnvironmentConstants.FINISH_TIME
                obj.updateagents();
                obj.Gui = obj.Gui.updategui(obj.Agents,obj.nAgents);
            end
            
        end
        
        function [] = createagent(obj,x0,y0,xg,yg)
            iAgent = obj.nAgents + 1;
            obj.Agents{iAgent} = RlcaAgent(x0,y0,xg,yg,iAgent);
            obj.nAgents = iAgent;
            
            
            obj.Gui.generateagentgraphic(obj.Agents{iAgent});
            
        end
        
        function obj = updateagents(obj)
            for iAgent = 1:obj.nAgents
                obj.Agents{iAgent} = obj.Agents{iAgent}.timestep();
            end
            
        end
        
    end
end

