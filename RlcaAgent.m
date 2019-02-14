classdef RlcaAgent
    %AGENT Summary of this class goes here
    % Author: Dale Collison
    
    %% RlcaAgent - Properties
    properties
        iAgent % Number allocated to the agent
        position = zeros(1,2) % Current position of the agent
        Velocity = struct('desired',[],'actual',[]) % Velocity information
        speed = []
        radius = [] % Collision radius of the agent
        heading = [] % Agent direction in degrees
        goal = zeros(1,2) % Goal position of the agent
        distanceToGoal = [] % Current distance to agent goal
        visionRadius = AgentConstants.VISION_RADIUS % Observable space around the agent
        Neighbours = struct('position',[],'velocity',[],'radius',[]) % Information on agents currently within its neighbourhood
        neighbourIds = []
        reachedGoal = 0 % Boolean to show if the agent has reached its intended goal
        collided = 0 % Boolean to show if the agent has been in a collision
        color % Colour of the agent on the plot
    end
    
    %% RlcaAgent - Public Methods
    methods (Access = public)
        function obj = RlcaAgent(x0,y0,xg,yg,iAgent)
            obj.radius = AgentConstants.RADIUS;
            obj.checkcoordinates(x0,y0,xg,yg);
            obj.iAgent = iAgent;
            obj.color = obj.getcolor();
            obj.position = [x0, y0];
            obj.goal = [xg, yg];
            obj.heading = obj.calcheading();
            obj.Velocity.actual = obj.calcvelocity();
            obj.speed = norm(obj.Velocity.actual);
            obj.distanceToGoal = obj.calcdistancetogoal();
        end
        
        function obj = timestep(obj)
            %             % Agent follows cursor
            %             A = get(0, 'PointerLocation');
            %             obj.goal.x = (A(1)-1306)/4.69;
            %             obj.goal.y = (A(2)-740)/4.69;
            
            obj.reachedGoal = obj.checkreachedgoal();
            if ~obj.reachedGoal
                obj.heading = obj.calcheading();
                obj.Velocity.actual = obj.calcvelocity();
                obj.speed = norm(obj.Velocity.actual);
                obj.position = calcposition(obj,EnvironmentConstants.TIME_STEP);
                obj.distanceToGoal = obj.calcdistancetogoal();
            else
                obj.speed = 0;
            end
            
        end
        
        function obj = addneighbour(obj,Neighbour)
            iNeighbour = find(obj.neighbourIds == Neighbour.iAgent);
            [nx, ny] = createarc(obj.heading + AgentConstants.VISION_ANGLE,...
                obj.heading - AgentConstants.VISION_ANGLE,obj.position(1),...
                obj.position(2),AgentConstants.VISION_RADIUS);
            isWithinNeighbourhood = inpolygon(Neighbour.position(1),...
                Neighbour.position(2),nx,ny);
            
            if ~isempty(iNeighbour) && ~isWithinNeighbourhood % If an existing neighbour has exited the neighbourhood
                obj.Neighbours.position(iNeighbour,:) = [];
                obj.Neighbours.velocity(iNeighbour,:) = [];
                obj.Neighbours.radius(iNeighbour) = [];
                obj.neighbourIds(iNeighbour) = [];
                disp(['Agent ' num2str(obj.iAgent) ' Lost Agent ' num2str(Neighbour.iAgent)])
            elseif isWithinNeighbourhood
                if isempty(iNeighbour)
                    iNeighbour = length(obj.neighbourIds)+1;
                    obj.neighbourIds(end+1) = Neighbour.iAgent;
                    disp(['Agent ' num2str(obj.iAgent) ' Detected Agent ' num2str(Neighbour.iAgent)])
                end
                obj.Neighbours.position(iNeighbour,:) = Neighbour.position;
                obj.Neighbours.velocity(iNeighbour,:) = Neighbour.Velocity.actual;
                obj.Neighbours.radius(iNeighbour,1) = Neighbour.radius;
            end
        end
        
    end
    
    %% RlcaAgent - Private Methods
    methods (Access = private)
        
        function distanceToGoal = calcdistancetogoal(obj)
            deltaX = abs(obj.position(1) - obj.goal(1));
            deltaY = abs(obj.position(2) - obj.goal(2));
            distanceToGoal = hypot(deltaX,deltaY);
        end
        
        function actualVelocity = calcvelocity(obj)
            actualVelocity(1) = AgentConstants.MAX_VELOCITY*cos(obj.heading);
            actualVelocity(2) = AgentConstants.MAX_VELOCITY*sin(obj.heading);
        end
        
        function position = calcposition(obj,deltaT)
            position(1) = obj.position(1) + (obj.speed*deltaT)*...
                cos(obj.heading);
            position(2) = obj.position(2) + (obj.speed*deltaT)*...
                sin(obj.heading);
        end
        
        function heading = calcheading(obj)
            deltaP = obj.goal - obj.position;
            heading = atan2(deltaP(2),deltaP(1));
        end
        
        function reachedGoal = checkreachedgoal(obj)
            deltaP = obj.goal - obj.position;
            
            deltaX = abs(deltaP(1));
            deltaY = abs(deltaP(2));
            
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

