classdef RlcaEnvironment < handle
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    %% RlcaEnvironment - Properties
    properties
        Agents = cell(0)
        nAgents = 0
        time = 0
        Gui
        agentsStatic
        nCollisions
        EventLog
    end
    
    %% RlcaEnvironment - Public Methods
    methods (Access = public)
        
        function obj = RlcaEnvironment()
            obj.EventLog = RlcaEventLog();
            obj.EventLog.createEvent('Initialising');
            obj.Gui = RlcaGui();
        end
        
        function obj = run(obj)
            pause(0.5)
            obj.EventLog.createEvent('Commencing run');
            t = EnvironmentConstants.START_TIME;
            while nnz(obj.agentsStatic) ~= obj.nAgents && t < EnvironmentConstants.MAX_TIME
                obj.time = t;
                obj = obj.updateagents();
                [obj.agentsStatic, obj.nCollisions] = obj.assessagents();
                obj.Gui = obj.Gui.updategui(obj.Agents,obj.nAgents);
                t = t + EnvironmentConstants.TIME_STEP;
                disp(obj.Agents{1}.Neighbours.position)
            end
            obj.EventLog.createEvent('Run complete');
        end
        
        function [] = createagent(obj,x0,y0,xg,yg)
            iAgent = obj.nAgents + 1;
            obj.Agents{iAgent} = RlcaAgent(x0,y0,xg,yg,iAgent);
            obj.nAgents = iAgent;
            
            obj.Gui = obj.Gui.generateagentgraphic(obj.Agents{iAgent});
        end
        
    end
    
    %% RlcaEnvironment - Privte Methods
    methods (Access = private)
        
        function obj = updateagents(obj)
            for iAgent = 1:obj.nAgents
                %TODO: Code to tell agent its neighbours
                for jAgent = 1:obj.nAgents
                    if iAgent ~= jAgent
                        obj.Agents{iAgent} = obj.Agents{iAgent}.addneighbour(obj.Agents{jAgent});
                    end
                end

                obj.Agents{iAgent} = obj.Agents{iAgent}.timestep();
            end
        end
        
        function [agentsStatic, nCollisions] = assessagents(obj)
            nCollisions = 0;
            agentsStatic = zeros(obj.nAgents,1);
            for iAgent = 1:obj.nAgents
                agentsStatic(iAgent) = obj.Agents{iAgent}.reachedGoal || obj.Agents{iAgent}.collided;
                if obj.Agents{iAgent}.collided
                    nCollisions = nCollisions + 1;
                end
            end
        end
        
    end
end

