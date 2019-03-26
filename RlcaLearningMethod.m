classdef RlcaLearningMethod < handle
    %RLCA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        StateSpace
        ActionSpace
        Q
        nStates = 0
        qHeatmap
    end
    
    methods
        function obj = RlcaLearningMethod()
            
            %% Create state space
            [x,y] = createarc(-AgentConstants.VISION_ANGLE+pi/2,AgentConstants.VISION_ANGLE+pi/2,0,0,AgentConstants.NEIGHBOURHOOD_RADIUS);
            
            for i = 1:13
                for j = 1:23
                    obj.StateSpace{i,j} = [5*j-60, 65-(5*i)];
                    if ~inpolygon(obj.StateSpace{i,j}(1),obj.StateSpace{i,j}(2),x,y)
                        obj.StateSpace{i,j} = [];
                    else
                        obj.nStates = obj.nStates + 1;
                    end
                end
            end
            
            %% Create action space
            X = -AgentConstants.MAX_SPEED:0.5:AgentConstants.MAX_SPEED;
            Y = -AgentConstants.MAX_SPEED:0.5:AgentConstants.MAX_SPEED;
            obj.ActionSpace = cell(length(X),length(Y));
            obj.Q = zeros(length(X),length(Y),obj.nStates);
            for iX = 1:length(X)
                for iY = 1:length(Y)
                    x = X(iX);
                    y = X(iY);
                    if norm([x,y]) <= AgentConstants.MAX_SPEED
                        obj.ActionSpace{iX,iY} = [x,y];
                        obj.Q(iX,iY,:) = 1;
                    end
                end
            end
            
            global A S
            A = obj.ActionSpace;
            S = obj.StateSpace;

        end
        
    end
end

