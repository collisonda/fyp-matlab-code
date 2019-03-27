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
        PastStates = -1*ones(1,20);
        actionId = -1;
        PastActions = -1*ones(1,20);
        epsilon = RLConstants.INITIAL_EPSILON
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
            
            obj.isAtGoal = obj.checkisatgoal();
            
            if ~obj.isAtGoal
                [obj.stateId] = obj.getstate();
                obj.PastStates = [obj.stateId, obj.PastStates(1:end-1)];
                
                [obj.actionId, obj.heading, obj.Velocity, obj.epsilon] = obj.calcaction();
                obj.PastActions = [obj.actionId, obj.PastActions(1:end-1)];
                
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
        
        function [actionId, heading, Velocity, epsilon] = calcaction(obj)
            global Q
            global A
            epsilon = obj.epsilon;
            if obj.stateId == 0 % No neighbours to worry about, go full speed at the goal.
                [actionId, heading, Velocity] = obj.calcgoalaction();
            else
                % Choose an action
                if (rand > epsilon) % Choose best option
                    sQ = Q(:,:,obj.stateId);
                    M = max(max(sQ));
                    [r,c] = find(sQ==M);
                    if length(r) > 1 % Multiple max Qs, randomly select one
                        selection = round(rand*length(r));
                        if selection == 0
                            selection = 1;
                        end
                        r = r(selection);
                        c = c(selection);
                    end
                    Velocity = A{r,c};                    
                else
                    % Choose random action
                    Velocity = [NaN, NaN];
                    while isnan(Velocity)
                        r = round(1 + rand*(size(A,1)-1));
                        c = round(1 + rand*(size(A,2)-1));
                        Velocity = A{r,c};
                    end
                end
                
                actionId = obj.getactionid(Velocity);
                    heading = atan2(Velocity(2),Velocity(1));
                
                epsilon = obj.epsilon * RLConstants.EPSILON_DECAY_RATE;
            end
        end
        
        function [stateId] = getstate(obj)
            global S
            if isempty(obj.neighbourIds)
                stateId = 0;
            else
                p = obj.position;
                np = obj.Neighbours.position;
                h = obj.heading;
                deltaP = obj.goal - obj.position;
                gh = atan2(deltaP(2),deltaP(1));
                
                npTranslated = obj.translateneighbourposition(p,np,gh);
                
                npTranslated = 10*round(npTranslated/10);
                
                if norm(npTranslated) > 60
                    signs = npTranslated./abs(npTranslated);
                    npTranslated = abs(npTranslated);
                    [~,idx] = max(npTranslated);
                    npTranslated(idx) = npTranslated(idx) - 10;
                    npTranslated = npTranslated.*signs;
                end
                
                matS = [S{:}];
                stateId = (strfind(matS,npTranslated) + 1)/2;
                if length(stateId) > 1
                    idx = floor(stateId) == stateId;
                    stateId = stateId(idx);
                end
                stateId = floor(stateId);
            end
        end
        
        function npTranslated = translateneighbourposition(obj,p,np,gh)
            % TODO: Translate the position of the neighbour to be relative
            % to the agent's position and in the direction of its goal.
            deltaPNeighbour = (np - p);
            deltaH = -gh+pi/2;
            
            rotationMatrix = [cos(deltaH), -sin(deltaH); sin(deltaH), cos(deltaH)];
            
            npTranslated = (rotationMatrix*deltaPNeighbour')';
            
        end
        
        function [actionId, heading, Velocity] = calcgoalaction(obj)
            deltaP = obj.goal - obj.position;
            headingVector = deltaP/norm(deltaP);
            heading = atan2(deltaP(2),deltaP(1));
            Velocity = round(2*headingVector*AgentConstants.MAX_SPEED)/2;
            if Velocity(1) == -4.5
                1;
            end
            if norm(Velocity) > AgentConstants.MAX_SPEED
                signs = Velocity./abs(Velocity);
                Velocity = abs(Velocity);
                Velocity = floor(Velocity);
                Velocity = Velocity.*signs;
            end
            actionId = obj.getactionid(Velocity);
        end
        
        function actionId = getactionid(~,Velocity)
            global A
            matA = [A{:}];
            actionId = (strfind(matA,Velocity) + 1)/2;
            if length(actionId) > 1
                idx = floor(actionId) == actionId;
                actionId = actionId(idx);
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
        
        function isAtGoal = checkisatgoal(obj)
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

