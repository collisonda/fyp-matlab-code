classdef RlcaAgent
    %AGENT Summary of this class goes here
    % Author: Dale Collison
    
    %% RlcaAgent - Properties
    properties
        iAgent % Number allocated to the agent
        position = zeros(1,2) % Current position of the agent
        Velocity = [0, 0]; % Velocity information
        preferredVelocity = [0,0];
        speed = AgentConstants.MAX_SPEED
        radius = [] % Collision radius of the agent
        heading = [] % Agent direction in degrees
        headingVector = []
        goal = zeros(1,2) % Goal position of the agent
        prevDistanceToGoal = []
        distanceToGoal = [] % Current distance to agent goal
        neighbourhoodRadius = AgentConstants.NEIGHBOURHOOD_RADIUS % Observable space around the agent
        Neighbours = struct('position',[],'velocity',[],'radius',[]) % Information on agents currently within its neighbourhood
        neighbourIds = []
        isAtGoal = 0 % Boolean to show if the agent has reached its intended goal
        hasCollided = 0 % Boolean to show if the agent has been in a collision
        color % Colour of the agent on the plot
        stateId = -1;
        PastStates = [-1 -1 1];
        actionId = -1;
        PastActions = [-1 -1 -1];
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
            [obj.heading, obj.headingVector] = obj.calcheading();
%             obj.Velocity = obj.calcvelocity();
%             obj.speed = norm(obj.Velocity);
            obj.distanceToGoal = obj.calcdistancetogoal();
        end
        
        function obj = timestep(obj)
            
            obj.isAtGoal = obj.checkisAtGoal();
            
            if ~obj.isAtGoal                
                [obj.stateId] = obj.getstate();
                obj.PastStates = [obj.stateId, obj.PastStates(1), obj.PastStates(2)];
                
                [obj.actionId, obj.heading, obj.Velocity] = obj.calcaction();
                obj.PastActions = [obj.actionId, obj.PastActions(1), obj.PastActions(2)]; 
                
                obj.speed = norm(obj.Velocity);
                
                obj.position = calcposition(obj,EnvironmentConstants.TIME_STEP);
                
                obj.prevDistanceToGoal = obj.distanceToGoal;
                obj.distanceToGoal = obj.calcdistancetogoal();
            else
                obj.speed = 0;
            end
            
        end
        
        function obj = addneighbour(obj,Neighbour)
            iNeighbour = find(obj.neighbourIds == Neighbour.iAgent);
            [nx, ny] = createarc(obj.heading + AgentConstants.VISION_ANGLE,...
                obj.heading - AgentConstants.VISION_ANGLE,obj.position(1),...
                obj.position(2),AgentConstants.NEIGHBOURHOOD_RADIUS);
            isWithinNeighbourhood = inpolygon(Neighbour.position(1),...
                Neighbour.position(2),nx,ny);
            
            if ~isempty(iNeighbour) && ~isWithinNeighbourhood % If an existing neighbour has exited the neighbourhood
                obj.Neighbours.position(iNeighbour,:) = [];
                obj.Neighbours.velocity(iNeighbour,:) = [];
                obj.Neighbours.radius(iNeighbour) = [];
                obj.neighbourIds(iNeighbour) = [];
                createevent(['Agent ' num2str(obj.iAgent) ' Lost Agent ' num2str(Neighbour.iAgent)])
            elseif isWithinNeighbourhood
                if isempty(iNeighbour)
                    iNeighbour = length(obj.neighbourIds)+1;
                    obj.neighbourIds(end+1) = Neighbour.iAgent;
                    createevent(['Agent ' num2str(obj.iAgent) ' Detected Agent ' num2str(Neighbour.iAgent)])
                end
                obj.Neighbours.position(iNeighbour,:) = Neighbour.position;
                obj.Neighbours.velocity(iNeighbour,:) = Neighbour.Velocity;
                obj.Neighbours.radius(iNeighbour,1) = Neighbour.radius;
            end
        end
        
    end
    
    %% RlcaAgent - Private Methods
    methods (Access = private)
        
        function [actionId, heading, Velocity] = calcaction(obj)
            if obj.stateId == 0 % No neighbours to worry about, go full speed at the goal.
                [actionId, heading, Velocity] = obj.calcgoalaction();
            else
                % Choose an action and evaluate
            end
        end
        
        function [stateId] = getstate(obj)
            global S
            if isempty(obj.neighbourIds)
                stateId = 0;                
            else
                stateId = 0; %debugging, remove
                
                p = obj.position;
                np = obj.Neighbours.position;
                h = obj.heading;
                deltaP = obj.goal - obj.position;
                gh = atan2(deltaP(2),deltaP(1));
                
                npTranslated = obj.translateneighbourposition(p,np,h,gh);
                
                npTranslated = 10*round(npTranslated/10);
                
                matS = [S{:}];
                stateId = strfind(matS,npTranslated);
                if length(stateId) > 1
                    stateId = stateId(2);
                end
            end
        end
        
        function npTranslated = translateneighbourposition(~,p,np,h,gh)
            % TODO: Translate the position of the neighbour to be relative
            % to the agent's position and in the direction of its goal.
            deltaPNeighbour = (p - np);
            deltaH = h-gh;
            
            rotationMatrix = [cos(deltaH), -sin(deltaH); sin(deltaH), cos(deltaH)];
            
            npTranslated = (rotationMatrix*deltaPNeighbour')';

        end
        
        function [actionId, heading, Velocity] = calcgoalaction(obj)
            deltaP = obj.goal - obj.position;
            headingVector = deltaP/norm(deltaP);
            heading = atan2(deltaP(2),deltaP(1));
            Velocity = round(2*headingVector*AgentConstants.MAX_SPEED)/2;
            actionId = obj.getactionid(Velocity);           
        end
        
        function actionId = getactionid(~,Velocity)
            global A
            matA = [A{:}];
            actionId = (strfind(matA,Velocity) + 1)/2;
            if length(actionId) > 1
                actionId = actionId(2);
            end
        end
        
        function distanceToGoal = calcdistancetogoal(obj)
            deltaX = abs(obj.position(1) - obj.goal(1));
            deltaY = abs(obj.position(2) - obj.goal(2));
            distanceToGoal = hypot(deltaX,deltaY);
        end
        
        function actualVelocity = calcvelocity(obj)
            actualVelocity = obj.speed*obj.headingVector;
        end
        
        function position = calcposition(obj,deltaT)
            position(1) = round(obj.position(1) + (obj.speed*deltaT)*...
                cos(obj.heading),4);
            position(2) = round(obj.position(2) + (obj.speed*deltaT)*...
                sin(obj.heading),4);
        end
        
        function [heading, headingVector] = calcheading(obj)
            deltaP = obj.goal - obj.position;
            headingVector = deltaP/norm(deltaP);
            heading = atan2(deltaP(2),deltaP(1));
            
        end
        
        function isAtGoal = checkisAtGoal(obj)
            deltaP = obj.goal - obj.position;
            
            deltaX = abs(deltaP(1));
            deltaY = abs(deltaP(2));
            
            isAtGoal = deltaX <= AgentConstants.GOAL_MARGIN &&...
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

