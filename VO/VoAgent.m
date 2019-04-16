classdef VoAgent
    %RLCAAGENT Summary of this class goes here
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
        goalHeading = []
        prevDistanceToGoal = []
        distanceToGoal = [] % Current distance to agent goal
        neighbourhoodRadius = AgentConstants.NEIGHBOURHOOD_RADIUS % Observable space around the agent
        Neighbours = struct('position',[],'velocity',[],'radius',[]) % Information on agents currently within its neighbourhood
        neighbourIds = []
        distToNeighbour = []
        isAtGoal = 0 % Boolean to show if the agent has reached its intended goal
        hasCollided = 0 % Boolean to show if the agent has been in a collision
        color % Colour of the agent on the plot
    end
    
    %% RlcaAgent - Public Methods
    methods (Access = public)
        function obj = VoAgent(x0,y0,xg,yg,iAgent)
            obj.radius = AgentConstants.RADIUS;
            obj.checkcoordinates(x0,y0,xg,yg);
            obj.iAgent = iAgent;
            obj.color = obj.getcolor();
            obj.position = [x0, y0];
            obj.goal = [xg, yg];
            
            
            deltaP = obj.goal - obj.position;
            obj.goalHeading = atan2(deltaP(2),deltaP(1));
            
            [obj.heading, obj.headingVector] = obj.calcheading();
            obj.Velocity = obj.headingVector*AgentConstants.MAX_SPEED;
            obj.distanceToGoal = obj.calcdistancetogoal();
        end
        
        function obj = timestep(obj)
            
            obj.isAtGoal = obj.checkisatgoal();
            
            if ~obj.isAtGoal
                [obj.heading, obj.Velocity] = obj.calcaction();
                obj.speed = norm(obj.Velocity);
                
                obj.position = calcposition(obj,EnvironmentConstants.TIME_STEP);
                
                [obj.distanceToGoal, obj.goalHeading] = obj.calcdistancetogoal();
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
            elseif isWithinNeighbourhood
                if isempty(iNeighbour)
                    iNeighbour = length(obj.neighbourIds)+1;
                    obj.neighbourIds(end+1) = Neighbour.iAgent;
                end
                obj.Neighbours.position(iNeighbour,:) = Neighbour.position;
                obj.Neighbours.velocity(iNeighbour,:) = Neighbour.Velocity;
                obj.Neighbours.radius(iNeighbour,1) = Neighbour.radius;
            end
        end
        
    end
    
    %% RlcaAgent - Private Methods
    methods (Access = private)
        
        function [heading, Velocity] = calcaction(obj)
            if isempty(obj.neighbourIds) % No neighbours to worry about, go full speed at the goal.
                [Velocity] = obj.calcgoalvelocity();
                heading = atan2(Velocity(2),Velocity(1));
            else %% do VO stuff
                [DesiredVelocity] = obj.calcgoalvelocity();
                DesiredHeading = obj.goalHeading;
                p_i = obj.position;
                v_i = obj.Velocity;
                r_i = obj.radius;
                tau_j = 0;
                VO = [];
                for iNeighbour = 1:length(obj.neighbourIds)
                    p_j = obj.Neighbours.position(iNeighbour,:);
                    v_j = obj.Neighbours.velocity(iNeighbour,:);
                    r_j = obj.Neighbours.radius(iNeighbour,1);
                    
                    VO_i = obj.defineVelocityObstacle(p_i,v_i,r_i,p_j,v_j,r_j,tau_j);
                    
                    
                    
                    VO_i.objectID = obj.neighbourIds(iNeighbour);
                    VO = [VO,VO_i];
                end
                
                % THE CLEAR PATH STRATEGY
                if DesiredVelocity(1) == 4.2
                    1;
                end
                [Velocity] = obj.strategy_clearPath(v_i,DesiredVelocity,VO);
                if isnan(Velocity(2))
                    1;
                end
%                 Velocity = obj.matchvelocity(Velocity);

                Velocity = Velocity';
                heading = atan2(Velocity(2),Velocity(1));
            end
            
        end
        
        function [projectedPoint,isOnTheRay] = pointProjectionToRay(obj,p,p0,v0)
            % INPUTS:
            % p  - Is the point to be projected.
            % p0,v0 - The line defining points
            % OUTPUTS:
            %
            projectedPoint = v0*v0'/(v0'*v0)*(p - p0) + p0;
            
            if v0'*(projectedPoint - p0)>0 % if on the ray
                isOnTheRay = logical(true);
            else
                isOnTheRay = logical(false);
            end
        end
        
        function [flag] = isInsideVO(obj,point,VO)
            % determine if the point p is inside the given VO
            % angle<halfOpenAngle; here numericalTolerance is an error tolarance
            
            flag = 0;
            VOtolerance = 1E-8;
            
            candidateVector = point - VO.apex;
            VOprojection   = norm(candidateVector)*cos(VO.openAngle/2);
            candProjection = VO.axisUnit'*candidateVector;
            projDiff = (candProjection - VOprojection);
            if projDiff > VOtolerance
                flag = 1;
            end
        end
        
        function [p_inter,isSuccessful] = twoRayIntersection2D(obj,P1,dP1,P2,dP2)
            % Find the intersection point between two 2D vectors. This
            % function isnt' interested if vertices are infront or behind
            % of the starting point.
            % INPUTS:
            % - P1,P2   - The ray defining points.
            % - dP1,dP2 - The ray unit directions.
            % OUTPUTS:
            % - p_inter - The 2D intersection point.
            
            assert(numel(P1) == 2,'Input must be 2D');
            assert(numel(P2) == 2,'Input must be 2D');
            
            % SOME SUFFICIENTLY SMALL VALUE FOR INTERSECTION
            isSuccessful = logical(false);   % Default to no intersection
            p_inter = NaN(2,1);              % Default to no intersection
            
            % THE 2D DETERMININANT
            div = dP1(2)*dP2(1) - dP1(1)*dP2(2);
            if div == 0
                disp('Lines are parallel');
                return % Lines are parallel
            end
            
            % THE SCALAR PROJECTIONS
            mua = (dP2(1)*(P2(2) - P1(2)) + dP2(2)*(P1(1) - P2(1))) / div;
            mub = (dP1(1)*(P2(2) - P1(2)) + dP1(2)*(P1(1) - P2(1))) / div;
            
            % POINTS MUST BE THE RESULT OF A POSITIVE INCREMENT OF THE VECTOR GRADIENT
            % (i.e, in the correct direction)
            if mua < 0 || mub < 0   % Intersections only occur in the direction of the vector
                return              % Lines do not intersect
            end
            
            % THE INTERSECTION POINT
            p_inter = P1 + mua*dP1;
            isSuccessful = logical(true);
        end
        
        % CLEAR PATH STRATEGY
        function [optimalVelocity] = strategy_clearPath(obj,v_i,desiredVelocity,VOset)
            % This function computes the optimal avoidance velocity using
            % the 'clear path' method of calculating the closest point to
            % the desired velocity on the surface of the VO set.
            
            % INPUT HANDLING
            if numel(VOset) == 0
                optimalVelocity = desiredVelocity;
                return
            end
            
            
            % ////////////// BUILD THE PROJECTION SET /////////////////////
            % We build a list of projection points, of closest proximity to
            % the desired velocity. There will be two projection points per
            % VO.
            projectionPoints = zeros(2,2*numel(VOset));
            isOnRayPoints    = ones(1,2*numel(VOset));
            a = 0;
            for VOnumA = 1:numel(VOset)
                % THE FIRST VERTEX EDGE
                [projections(:,1),isOnRay(1)] = obj.pointProjectionToRay(desiredVelocity',VOset(VOnumA).apex,VOset(VOnumA).leadingEdgeUnit);
                % THE SECOND VERTEX EDGE
                [projections(:,2),isOnRay(2)] = obj.pointProjectionToRay(desiredVelocity',VOset(VOnumA).apex,VOset(VOnumA).trailingEdgeUnit);
                
                % COLLECT THE PROJECTIONS POINTS
                % The projections of 'v_a' on both the leadingEdgeUnit, trailingEdgeUnit
                isOnRayPoints((1 + a*VOnumA):(2 + a*VOnumA)) = isOnRay;          % CONCATINATE THE IS ON RAY
                projectionPoints(:,(1 + a*VOnumA):(2 + a*VOnumA)) = projections; % STORE ALL PROJECTION POINTS
                a = a + 1;
            end
            
            % /////////// BUILD THE INTERSECTION POINT SET ////////////////
            % GET THE INTERSECTIONS BETWEEN TWO SETS OF LEADING & TRAILING
            % EDGES
            VOsum = numel(VOset);
            intersectionFlags  = ones(1,4*VOsum*(VOsum-1)/2);
            intersectionPoints = zeros(2,4*VOsum*(VOsum-1)/2);
            a = 0;
            for VOnum_i = 1:numel(VOset)
                for VOnum_j = 1:numel(VOset)
                    if VOnum_i == VOnum_j
                        continue % Skip self comparison (also omits singular VO condition)
                    end
                    pIntersect = zeros(2,4);
                    % LEADING - LEADING
                    [pIntersect(:,1),validIntersect(1)] = obj.twoRayIntersection2D(...
                        VOset(VOnum_i).apex,...
                        VOset(VOnum_i).leadingEdgeUnit,...
                        VOset(VOnum_j).apex,...
                        VOset(VOnum_j).leadingEdgeUnit);
                    % LEADING - TRAILING
                    [pIntersect(:,2),validIntersect(2)] = obj.twoRayIntersection2D(...
                        VOset(VOnum_i).apex,...
                        VOset(VOnum_i).leadingEdgeUnit,...
                        VOset(VOnum_j).apex,...
                        VOset(VOnum_j).trailingEdgeUnit);
                    % TRAILING - LEADING
                    [pIntersect(:,3),validIntersect(3)] = obj.twoRayIntersection2D(...
                        VOset(VOnum_i).apex,...
                        VOset(VOnum_i).trailingEdgeUnit,...
                        VOset(VOnum_j).apex,...
                        VOset(VOnum_j).leadingEdgeUnit);
                    % TRAILING - TRAILING
                    [pIntersect(:,4),validIntersect(4)] = obj.twoRayIntersection2D(...
                        VOset(VOnum_i).apex,...
                        VOset(VOnum_i).trailingEdgeUnit,...
                        VOset(VOnum_j).apex,...
                        VOset(VOnum_j).trailingEdgeUnit);
                    
                    % There are four intersections per pair of VO.
                    
                    % RETAIN THE POINTS & FLAGS
                    intersectionFlags(:,(1 + 4*a):4*(1 + a)) = validIntersect; % If the corresponding point was a valid intersection
                    intersectionPoints(:,(1 + 4*a):4*(1 + a)) = pIntersect;    % The intersection point array
                    a = a + 1;
                end
            end
            
            % ASSSESS THE COLLECTIVE POINT SET AGAINSTS THE VO SET
            % All valid projections and intersection must be compared
            % against thw VO set.
            
            % OMIT NON-VALID PROJECTIONS
            validProjectionPoints = projectionPoints(:,(isOnRayPoints == 1));    % Get only the projections where the points are on rays
            % REMOVE ANY NON-INTERSECTIONS
            validIntersectionPoints = intersectionPoints(:,(intersectionFlags == 1)); % Are valid intersections
            
            % CONSIDER THE CURRENT VELOCITY IN THE CANDIDATE SET
            collectivePoints = [desiredVelocity',validProjectionPoints,validIntersectionPoints]; % <<< TO BE CONFIRMED
            collectivePoints = unique(collectivePoints','rows');           % Remove repeat candidates
            collectivePoints = collectivePoints';
            
            % ///////// CHECK EACH POINT AGAINST THE VO SET ///////////////
            VOflagVector = zeros(1,size(collectivePoints,2));
            for candidate = 1:size(collectivePoints,2)
                for VOnum_i = 1:numel(VOset)
                    % DETERMINE WHETHER THE POINT BELONGS TO ANY VO
                    if VOflagVector(candidate) || obj.isInsideVO(collectivePoints(:,candidate),VOset(VOnum_i))
                        VOflagVector(candidate) = 1;
                    end
                end
            end
            
            % REMOVE THE VO-INVALIDATED CANDIDATE POINTS
            candidatesOutsideVO = collectivePoints(:,VOflagVector ~= 1);
            
            % ///// CHOOSE OPTIMAL VELOCITY FROM THE CANDIDATE POINTS /////
            optimalMetricDistance = inf;  % Metric of optimality
            compareVelocity = desiredVelocity;
            
            % DEFAULT VELOCITY
            optimalVelocity = zeros(2,1);
            
            if size(candidatesOutsideVO,2) > 0
                % ASSESS VELOCITIES AGAINST THE DESIRED VELOCITY
                for k = 1:size(candidatesOutsideVO,2)
                    dis = norm(candidatesOutsideVO(:,k) - compareVelocity);
                    if dis < optimalMetricDistance
                        optimalVelocity = candidatesOutsideVO(:,k);
                        optimalMetricDistance = dis;
                    end
                end
            elseif isempty(candidatesOutsideVO)
                % IN THE EVENT THERE ARE NO VALID VELOCITIES
                warning('There is no feasible velocity!');
                optimalVelocity = zeros(2,1);
            end
        end
        
        function [VO] = defineVelocityObstacle(obj,p_i,v_i,r_i,p_j,v_j,r_j,tau)
            p_i = [p_i';0];
            v_i = [v_i';0];
            p_j = [p_j';0];
            v_j = [v_j';0];
            lambda_ij = p_j - p_i;
            r_c = r_i + r_j + 4;
            mod_lambda_ij    = norm(lambda_ij);                            % Scalar relative seperation
            unit_lambda_ij   = lambda_ij/mod_lambda_ij;                    % Unit relative position
            % DEFINE THE ANGLE REFERENCE AXES
            referenceAxis = [1;0;0];
            planarNormal = cross(referenceAxis,unit_lambda_ij);
            if sum(planarNormal) == 0
                planarNormal = cross([0;0;1],unit_lambda_ij);                % Axes then align with the aerospace convention
            end
            % CALCUATE THE OPEN ANGLE OF THE CONE
            halfAlpha = asin(r_c/mod_lambda_ij);
            halfAlpha = real(halfAlpha);
            
            leadingTangentVector = obj.rodriguesRotation(lambda_ij,planarNormal,halfAlpha);
            
            % CALCULATE THE AXIS PROJECTION
            VOaxis = (dot(leadingTangentVector,lambda_ij)/mod_lambda_ij^2)*lambda_ij; % Define the tangent projection on AB
            axisLength = norm(VOaxis);
            axisUnit = VOaxis/axisLength;
            
            % DEFINE THE LEADING TANGENT VECTOR
            [leadingTangentVector] = obj.rodriguesRotation(VOaxis,planarNormal,halfAlpha);
            unit_leadingTangent = leadingTangentVector/norm(leadingTangentVector);
            
            % DEFINE THE TRAILING VECTORS (TRAILING)
            [trailingTangentVector] = obj.rodriguesRotation(VOaxis,planarNormal,-halfAlpha);
            unit_trailingTangent = trailingTangentVector/norm(trailingTangentVector);
            
            % ESTABLISH DIRECTION OF THE AGENT
            leadingTest = unit_leadingTangent'*(v_i - v_j) > unit_trailingTangent'*(v_i - v_j);
            
            
            if leadingTest
                isVaLeading = 1;
            else
                isVaLeading = 0;
            end
            
            % //////// DEFINE VELOCITY OBSTACLE PARAMETERS ////////////////
            VO = struct('apex',v_j,...
                'axisUnit',axisUnit,...
                'axisLength',mod_lambda_ij,...
                'openAngle',2*halfAlpha,...
                'leadingEdgeUnit',unit_leadingTangent,...
                'trailingEdgeUnit',unit_trailingTangent,...
                'isVaLeading',isVaLeading,...
                'isVaInsideCone',0,...
                'truncationTau',tau,...
                'truncationCircleCenter',(p_j - p_i)/tau + v_j,...
                'truncationCircleRadius',(r_i + r_j)/tau);
            % /////////////////////////////////////////////////////////////
            % IS Va INSIDE THE VO
            [VO.isVaInsideCone] = obj.isInCone(VO,v_i);
            
            
            % ALTER THE VO PARAMETERS
            VO.apex = VO.apex(1:2,1);
            VO.axisUnit = VO.axisUnit(1:2,1);
            VO.leadingEdgeUnit = VO.leadingEdgeUnit(1:2,1);
            VO.trailingEdgeUnit = VO.trailingEdgeUnit(1:2,1);
            VO.truncationCircleCenter = VO.truncationCircleCenter(1:2,1);
        end
        
        function [flag] = isInCone(obj,VO,probePoint)
            % INPUTS:
            % v_b
            % mod_VOaxis
            % unit_VOaxis
            % coneOpenAngle - Cone open angle
            % probePoint    - Test point
            
            % Compare the angle made between the probeVector and the
            % unitVO axis. Is less than alpha? else not in cone.
            
            % FROM THE APEX TO THE PROBE POINT
            probeVector = probePoint - VO.apex;                            % Define validation vector
            [unit_probeVector] = probeVector/norm(probeVector);
            
            probeDot = dot(unit_probeVector,VO.axisUnit);
            theta = real(acos(probeDot));                                  % The angle between cone axis and probe vector
            
            % CHECK POINT RELATION TO CONE GEOMETRY
            flag = 0;
            conditionA = theta - VO.openAngle/2 < 1e-5;             % With an angle tolerance
            conditionB = probeDot > 0;                                     % In the same direction as the axis vector
            if conditionA && conditionB                                    % Half the cone's open angle
                flag = 1;
            end
        end
        
        function [v_rotated] = rodriguesRotation(~,u,k,theta)
            % v - Vector to be rotated
            % k - Is the rotation axis
            % Theta - The angle the vector is to be rotated through
            
            assert(numel(u) == 3,'Rotation vector must be of size [3x1].')
            assert(numel(k) == 3,'The rotation axis must be of size [3x1]');
            assert(numel(theta) == 1,'The rotation angle %.0f must be a scalar',theta);
            
            [m,n] = size(u);
            if (m ~= 3 && n ~= 3)
                error('input vector is/are not three dimensional')
            end
            if (size(u) ~= size(k))
                error('rotation vector v and axis k have different dimensions')
            end
            
            k = k/sqrt(k(1)^2 + k(2)^2 + k(3)^2); % normalize rotation axis
            No = numel(u)/3; % number of vectors in array
            v_rotated = u; % initialize rotated vector array
            if ( n == 3 )
                crosskv = u(1,:); % initialize cross product k and v with right dim.
                for i = 1:No
                    crosskv(1) = k(2)*u(i,3) - k(3)*u(i,2);
                    crosskv(2) = k(3)*u(i,1) - k(1)*u(i,3);
                    crosskv(3) = k(1)*u(i,2) - k(2)*u(i,1);
                    v_rotated(i,:) = cos(theta)*u(i,:) + (crosskv)*sin(theta)...
                        + k*(dot(k,u(i,:)))*(1 - cos(theta));
                end
            else % if m == 3 && n ~= 3
                crosskv = u(:,1); % initialize cross product k and v with right dim.
                for i = 1:No
                    crosskv(1) = k(2)*u(3,i) - k(3)*u(2,i);
                    crosskv(2) = k(3)*u(1,i) - k(1)*u(3,i);
                    crosskv(3) = k(1)*u(2,i) - k(2)*u(1,i);
                    v_rotated(:,i) = cos(theta)*u(:,i) + (crosskv)*sin(theta)...
                        + k*(dot(k,u(:,i)))*(1 - cos(theta));
                end
            end
        end
        
        function velocityTranslated = translatevelocity(~,Velocity,theta)
            if nnz(isnan(Velocity)) > 0
                1;
            end
            rotationMatrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            
            velocityTranslated = (rotationMatrix*Velocity')';
            
        end
        
        function npTranslated = translateneighbourposition(~,p,np,gh)
            deltaPNeighbour = (np - p);
            deltaH = -gh+pi/2;
            
            rotationMatrix = [cos(deltaH), -sin(deltaH); sin(deltaH), cos(deltaH)];
            
            npTranslated = (rotationMatrix*deltaPNeighbour')';
            
        end
        
        function [Velocity] = calcgoalvelocity(obj)
            
            deltaP = obj.goal - obj.position;
            headingVector = deltaP/norm(deltaP);
            Velocity = headingVector*AgentConstants.MAX_SPEED;
            theta = -atan2(deltaP(2),deltaP(1))+pi/2;
            Velocity = round(Velocity,6);
            Velocity = obj.matchvelocity(Velocity);
            
        end
        
        function [Velocity] = matchvelocity(~,exactVelocity)
            v = linspace(-AgentConstants.MAX_SPEED,AgentConstants.MAX_SPEED,101);
            v = round(v,3);
            Velocity = interp1(v,v,exactVelocity,'nearest');
        end
        
        function [distanceToGoal, goalHeading] = calcdistancetogoal(obj)
            deltaX = abs(obj.position(1) - obj.goal(1));
            deltaY = abs(obj.position(2) - obj.goal(2));
            distanceToGoal = hypot(deltaX,deltaY);
            deltaP = obj.goal - obj.position;
            goalHeading = atan2(deltaP(2),deltaP(1));
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

