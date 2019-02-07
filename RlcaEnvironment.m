classdef RlcaEnvironment < handle
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Agents = cell(0);
        nAgents = 0;
        time = 0;
    end
    
    methods
        function obj = RlcaEnvironment()
            
            
        end
        
        function obj = timeStep(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            for iAgent = 1:obj.nAgents
               obj.Agents(iAgent).timeStep(EnvironmentConstants.stepSize); 
            end
        end
        
        function [] = createAgent(obj,x0,y0,xg,yg)
           iAgent = obj.nAgents + 1;
           obj.Agents{iAgent} = RlcaAgent(x0,y0,xg,yg,iAgent);
           obj.nAgents = iAgent;
        end
    end
end

