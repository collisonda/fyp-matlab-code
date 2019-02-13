classdef RlcaAgent
    %AGENT Summary of this class goes here
    % Author: Dale Collison
    
    %% RlcaAgent - Properties
    properties
        iAgent              % Number allocated to the agent
        Position            = struct('x',[],'y',[]) % Current position of the agent
        Velocity            = struct('preferred',[],'actual',6,'max',[]) % Velocity information
        radius              % Collision radius of the agent
        heading             = [] % Agent direction in degrees
        Goal                = struct('x',[],'y',[]) % Goal position of the agent
        distanceToGoal      = [] % Current distance to agent goal
        neighbourhoodRadius = AgentConstants.NEIGHBOURHOOD_RADIUS; % Observable space around the agent
        Neighbours          = [] % Information on agents currently within its neighbourhood
        reachedGoal         = 0; % Boolean to show if the agent has reached its intended goal
        collided            = 0; % Boolean to show if the agent has been in a collision
        color               % Colour of the agent on the plot
    end
    
    %% RlcaAgent - Public Methods
    methods (Access = public)
        function obj = RlcaAgent(x0,y0,xg,yg,iAgent)
            obj.radius = AgentConstants.RADIUS;
            obj.checkcoordinates(x0,y0,xg,yg);
            obj.iAgent          = iAgent;
            obj.color           = obj.getcolor();
            obj.Position.x      = x0;
            obj.Position.y      = y0;
            obj.Goal.x          = xg;
            obj.Goal.y          = yg;
            obj.heading         = obj.calcheading();
            obj.distanceToGoal  = obj.calcdistancetogoal();
        end
        
        function obj = timestep(obj)
            % Agent follows cursor
            A = get(0, 'PointerLocation');
            obj.Goal.x = (A(1)-1306)/4.69;
            obj.Goal.y = (A(2)-740)/4.69;
            %
            
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
        
    end
    
    %% RlcaAgent - Private Methods
    methods (Access = private)
        
        function distanceToGoal = calcdistancetogoal(obj)
            deltaX = abs(obj.Position.x - obj.Goal.x);
            deltaY = abs(obj.Position.y - obj.Goal.y);
            distanceToGoal = hypot(deltaX,deltaY);
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
        
        function checkcoordinates(~,x0,y0,xg,yg)
            isWithinX = nnz(([x0,xg] < EnvironmentConstants.X_BOUNDARY(1))...
                | [x0,xg] > EnvironmentConstants.X_BOUNDARY(2)) == 0;
            isWithinY = nnz(([y0,yg] < EnvironmentConstants.Y_BOUNDARY(1))...
                | [y0,yg] > EnvironmentConstants.Y_BOUNDARY(2)) == 0;
            if ~isWithinX || ~isWithinY
                error('Coordinates outside of environment boundaries.')
            end
        end
        
    end
    
end

