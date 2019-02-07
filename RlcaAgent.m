classdef RlcaAgent
    %AGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        iAgent  % Number allocated to the agent
        Position = struct('x',[],'y',[]) % Current position of the agent
        Velocity = struct('preferred',[],'actual',0,'max',[]) % Velocity information
        radius = AgentConstants.radius; % Collision radius of the agent
        heading = [] % Agent direction in degrees
        Goal = struct('x',[],'y',[]) % Goal position of the agent
        distanceToGoal = [] % Current distance to agent goal
        neighbourhoodRadius = AgentConstants.neighbourhoodRadius; % Observable space around the agent
        Neighbours = [] % Information on agents currently within its neighbourhood
    end
    
    methods
        function obj = RlcaAgent(x0,y0,xg,yg)
            obj.iAgent = obj.getIAgent();
            obj.Position.x = x0;
            obj.Position.y = y0;
            obj.Goal.x = xg;
            obj.Goal.y = yg;
            
            obj.distanceToGoal = obj.calcDistanceToGoal();
        end
        
        function distanceToGoal = calcDistanceToGoal(obj)
            deltaX = abs(obj.Position.x - obj.Goal.x);
            deltaY = abs(obj.Position.y - obj.Goal.y);
            distanceToGoal = hypot(deltaX,deltaY);
        end
        
        function Position = calcHeading(obj)
            Position.x = obj.Position.x + (obj.Velocity.actual*deltaT)*cos(deg2rad(obj.heading));
            Position.y = obj.Position.y + (obj.Velocity.actual*deltaT)*sin(deg2rad(obj.heading));
        end
        
        function obj = timeStep(obj,deltaT)
            obj.Position = calcPosition(obj,deltaT);
            obj.distanceToGoal = obj.calcDistanceToGoal();
            %obj.Neighbours = obj.assessNeighbours();
        end
        
        function Position = calcPosition(obj,deltaT)
            Position.x = obj.Position.x + (obj.Velocity.actual*deltaT)*cos(deg2rad(obj.heading));
            Position.y = obj.Position.y + (obj.Velocity.actual*deltaT)*sin(deg2rad(obj.heading));
        end
        
        function Neighbours = assessNeighbours(obj)
            
        end
        
    end
    
    methods (Static)
        function iAgent = getIAgent()
           global nAgents
           iAgent = nAgents + 1;
           nAgents = iAgent;
        end
        
    end
end

