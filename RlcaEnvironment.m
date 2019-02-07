classdef RlcaEnvironment
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Agents
        nAgents = 0;
        time
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
    end
end

