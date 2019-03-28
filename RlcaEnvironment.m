classdef RlcaEnvironment < handle
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    %% RlcaEnvironment - Properties
    properties
        Agents = cell(0)
        nAgents = 0
        time = 0
        Gui
        isAgentStatic
        collision = 0
        goalsReached = 0
        guiOn
        eventsOn
    end
    
    %% RlcaEnvironment - Public Methods
    methods (Access = public)
        
        function obj = RlcaEnvironment(guiOn,eventsOn)
            
            obj.guiOn = guiOn;
            obj.eventsOn = eventsOn;
            if obj.guiOn
                obj.Gui = RlcaGui();
            end
            if obj.eventsOn
                createevent('Initialising');
            end
        end
        
        function obj = runsimulation(obj)
            pause(0.5)
            tic
            if obj.eventsOn
                createevent();
                createevent('Commencing run');
            end
            t = EnvironmentConstants.START_TIME;
            while nnz(obj.isAgentStatic) ~= obj.nAgents && t < EnvironmentConstants.MAX_TIME && obj.collision == 0
                obj.time = t;
                obj = obj.updateagents();
                [obj.isAgentStatic, obj.collision, obj.goalsReached] = obj.assessagents();
                obj.assessq();
                if obj.guiOn
                    obj.Gui = obj.Gui.updategui(obj.Agents,obj.nAgents);
                end
                t = t + EnvironmentConstants.TIME_STEP;
            end
            if obj.eventsOn
                createevent('Run complete');
                createevent();
            end
            tElapsed = toc;
            tElapsedSim = t;
            if obj.eventsOn
                createevent(['Real Time Elapsed        ' num2str(tElapsed) ' s']);
                createevent(['Simulation Time Elapsed  ' num2str(tElapsedSim) ' s']);
                createevent(['Real:Simulation Ratio    ' num2str(tElapsedSim/tElapsed)]);
            end
        end
        
        function [] = createagent(obj,x0,y0,xg,yg)
            iAgent = obj.nAgents + 1;
            obj.Agents{iAgent} = RlcaAgent(x0,y0,xg,yg,iAgent);
            obj.nAgents = iAgent;
            if obj.guiOn
                obj.Gui = obj.Gui.generateagentgraphic(obj.Agents{iAgent});
            end
        end
        
    end
    
    %% RlcaEnvironment - Private Methods
    methods (Access = private)
        
        function obj = updateagents(obj)
            for iAgent = 1:obj.nAgents
                %TODO: Code to tell agent its neighbours
                for jAgent = 1:obj.nAgents
                    if iAgent ~= jAgent
                        obj.Agents{iAgent} = obj.Agents{iAgent}.addneighbour(obj.Agents{jAgent});
                    end
                end
                
                obj.Agents{iAgent} = obj.Agents{iAgent}.timestep();
            end
        end
        
        function [isAgentStatic, collision, goalsReached] = assessagents(obj)
            goalsReached = 0;
            isAgentStatic = zeros(obj.nAgents,1);
            for iAgent = 1:obj.nAgents
                isAgentStatic(iAgent) = obj.Agents{iAgent}.isAtGoal;
            end
            
            if nnz(isAgentStatic) == obj.nAgents
               goalsReached = 1; 
            end
            
            agent1Pos = obj.Agents{1}.position;
            agent2Pos = obj.Agents{2}.position;
            r = AgentConstants.RADIUS;
            
            [x,~] = circcirc(agent1Pos(1),agent1Pos(2),r,agent2Pos(1),agent2Pos(2),r);
            
            if ~isnan(x) % If there is an intersection point, the agents have collided
                collision = 1;
            else
                collision = 0;
            end
        end
        
        function [obj] = assessq(obj)
            global Q
            global A
            reward = zeros(1,2);
            if obj.collision
                reward = [RLConstants.COLLISION_PENALTY, RLConstants.COLLISION_PENALTY];
            elseif obj.goalsReached
                reward = [RLConstants.GOAL_REWARD, RLConstants.GOAL_REWARD];
            else
                for iAgent = 1:obj.nAgents
                    reward(iAgent) = obj.calcreward(obj.Agents{iAgent});
                end
            end
            
            matA = [A{:}];
            
            for iAgent = 1:obj.nAgents
                Agent = obj.Agents{iAgent};
                
                for iAction = 1:length(Agent.PastActions)
                    iState = iAction;
                    actionId = Agent.PastActions(iAction);
                    stateId = Agent.PastStates(iState);
                    if stateId > 0
                        sQ = Q(:,:,stateId);
                        factor = (RLConstants.BASE_FACTOR + RLConstants.DISCOUNT_FACTOR^iAction);
                        weightedReward = ((sQ(actionId) * (2 - factor)) + (factor * reward(iAgent)))/2;
                        sQ(actionId) = weightedReward;
                        Q(:,:,stateId) = sQ;
                    end
                end
                
            end
            
        end
        
        function R = calcreward(~,Agent)
            R = 0;
            % Calculate goal distance reward
            if ~isempty(Agent.prevDistanceToGoal)
                maxDeltaGoalDistance = AgentConstants.MAX_SPEED*EnvironmentConstants.TIME_STEP;
                deltaGoalDistance = Agent.prevDistanceToGoal - Agent.distanceToGoal;
                gdR = RLConstants.GOAL_DISTANCE_WEIGHT*(deltaGoalDistance/maxDeltaGoalDistance);
                R = R + gdR;
            end
            
            % Calculate similar heading reward
            if ~isempty(Agent.prevHeading)
                maxDeltaHeading = pi/2;
                deltaHeading = abs(wrapToPi(Agent.prevHeading - Agent.heading));
                shR = (1 - (deltaHeading/maxDeltaHeading)) * RLConstants.SIMILAR_HEADING_WEIGHT;
                R = R + shR;
            end
        end
        
        
    end
end

