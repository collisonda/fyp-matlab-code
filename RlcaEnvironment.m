classdef RlcaEnvironment < handle
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    %% RlcaEnvironment - Properties
    properties
        Agents = cell(0)
        nAgents = 0
        time = 0
        Gui
        isAgentStatic
        nCollisions
    end
    
    %% RlcaEnvironment - Public Methods
    methods (Access = public)
        
        function obj = RlcaEnvironment()
            createevent('Initialising');
            obj.Gui = RlcaGui();
        end
        
        function obj = runsimulation(obj)
            pause(0.5)
            tic
            createevent();
            createevent('Commencing run');
            t = EnvironmentConstants.START_TIME;
            while nnz(obj.isAgentStatic) ~= obj.nAgents && t < EnvironmentConstants.MAX_TIME
                obj.time = t;
                obj = obj.updateagents();
                [obj.isAgentStatic, obj.nCollisions] = obj.assessagents();
                obj.Gui = obj.Gui.updategui(obj.Agents,obj.nAgents);
                t = t + EnvironmentConstants.TIME_STEP;
            end
            createevent('Run complete');
            createevent();
            tElapsed = toc;
            tElapsedSim = t;
            createevent(['Real Time Elapsed        ' num2str(tElapsed) ' s']);
            createevent(['Simulation Time Elapsed  ' num2str(tElapsedSim) ' s']);
            createevent(['Real:Simulation Ratio    ' num2str(tElapsedSim/tElapsed)]);
        end
        
        function [] = createagent(obj,x0,y0,xg,yg)
            iAgent = obj.nAgents + 1;
            obj.Agents{iAgent} = RlcaAgent(x0,y0,xg,yg,iAgent);
            obj.nAgents = iAgent;
            
            obj.Gui = obj.Gui.generateagentgraphic(obj.Agents{iAgent});
        end
        
    end
    
    %% RlcaEnvironment - Private Methods
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
        
        function [isAgentStatic, nCollisions] = assessagents(obj)
            nCollisions = 0;
            isAgentStatic = zeros(obj.nAgents,1);
            for iAgent = 1:obj.nAgents
                isAgentStatic(iAgent) = obj.Agents{iAgent}.isAtGoal || obj.Agents{iAgent}.hasCollided;
                if obj.Agents{iAgent}.hasCollided
                    nCollisions = nCollisions + 1;
                end
            end
        end
        
    end
end

