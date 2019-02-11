classdef RlcaAgent
    %AGENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        iAgent  % Number allocated to the agent
        Position = struct('x',[],'y',[]) % Current position of the agent
        Velocity = struct('preferred',[],'actual',6,'max',[]) % Velocity information
        radius = AgentConstants.RADIUS; % Collision radius of the agent
        heading = [] % Agent direction in degrees
        Goal = struct('x',[],'y',[]) % Goal position of the agent
        distanceToGoal = [] % Current distance to agent goal
        neighbourhoodRadius = AgentConstants.NEIGHBOURHOOD_RADIUS; % Observable space around the agent
        Neighbours = [] % Information on agents currently within its neighbourhood
        reachedGoal = 0;
        color
    end
    
    methods
        function obj = RlcaAgent(x0,y0,xg,yg,iAgent)
            obj.iAgent = iAgent;
            obj.color = obj.getcolor();
            obj.Position.x = x0;
            obj.Position.y = y0;
            obj.Goal.x = xg;
            obj.Goal.y = yg;
            obj.heading = obj.calcheading();
            obj.distanceToGoal = obj.calcdistancetogoal();
        end
        
        function distanceToGoal = calcdistancetogoal(obj)
            deltaX = abs(obj.Position.x - obj.Goal.x);
            deltaY = abs(obj.Position.y - obj.Goal.y);
            distanceToGoal = hypot(deltaX,deltaY);
        end
        
        function obj = timestep(obj)
            obj.reachedGoal = obj.checkreachedgoal();
            if ~obj.reachedGoal
                obj.heading = obj.calcheading();
                obj.Position = calcposition(obj,EnvironmentConstants.TIME_STEP);
                obj.distanceToGoal = obj.calcdistancetogoal();
                %obj.Neighbours = obj.assessNeighbours();
            else
                obj.Velocity.actual = 0;
            end
        end
        
        function Position = calcposition(obj,deltaT)
            Position.x = obj.Position.x + (obj.Velocity.actual*deltaT)*...
                cos(obj.heading);
            Position.y = obj.Position.y + (obj.Velocity.actual*deltaT)*...
                sin(obj.heading);
        end
        
        function heading = calcheading(obj)
            x = obj.Position.x;
            y = obj.Position.y;
            xg = obj.Goal.x;
            yg = obj.Goal.y;
            heading = atan2(yg-y,xg-x);
        end
        
        function Neighbours = assessneighbours(obj)
            %TODO
        end
        
        function reachedGoal = checkreachedgoal(obj)
            x = obj.Position.x;
            y = obj.Position.y;
            xg = obj.Goal.x;
            yg = obj.Goal.y;
            
            deltaX = abs(xg-x);
            deltaY = abs(yg-y);
            
            reachedGoal = deltaX <= AgentConstants.GOAL_MARGIN &&...
                deltaY <= AgentConstants.GOAL_MARGIN;
            
        end
        
        function color = getcolor(obj)
           iColor = obj.iAgent;
           if iColor > 9
              iColor = mod(iColor,9); 
           end
           color = AgentConstants.COLOR_ORDER(iColor,1:end); 
        end
        
    end
    
end

