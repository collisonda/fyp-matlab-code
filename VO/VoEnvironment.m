classdef VoEnvironment < handle
    %RLCAENVIRONMENT Summary of this class goes here
    %   Author: Dale Collison
    
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
        tOptimal
    end
    
    %% RlcaEnvironment - Public Methods
    methods (Access = public)
        
        function obj = VoEnvironment(guiOn,Scenario)
            obj.tOptimal = 30;
            obj.guiOn = guiOn;
            obj.eventsOn = 0;
            if obj.guiOn
                obj.Gui = VoGui();
            end
            if obj.eventsOn
                createevent('Initialising');
            end
            
            for iAgent = 1:size(Scenario,1)
                obj.createagent(Scenario(iAgent,1),Scenario(iAgent,2),Scenario(iAgent,3),Scenario(iAgent,4));
            end
            
        end
        
        function [goalsReached, tElapsedSim] = runsimulation(obj)
            
            if obj.eventsOn
                createevent();
                createevent('Commencing run');
            end
            
            t = EnvironmentConstants.START_TIME - EnvironmentConstants.TIME_STEP;
            
            while nnz(obj.isAgentStatic) ~= obj.nAgents && t < EnvironmentConstants.MAX_TIME && nnz(obj.collision) < 1
                t = t + EnvironmentConstants.TIME_STEP;
                obj.time = t;
                obj = obj.updateagents();
                [obj.isAgentStatic, obj.collision, obj.goalsReached] = obj.assessagents();
                
                if obj.guiOn
                    obj.Gui = obj.Gui.updategui(obj.Agents,obj.nAgents);
                end
                
            end

            tElapsedSim = t;
            goalsReached = obj.goalsReached;
        end
        
        function [] = createagent(obj,x0,y0,xg,yg)
            iAgent = obj.nAgents + 1;
            obj.Agents{iAgent} = VoAgent(x0,y0,xg,yg,iAgent);
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
                for jAgent = 1:obj.nAgents
                    if iAgent ~= jAgent
                        obj.Agents{iAgent} = obj.Agents{iAgent}.addneighbour(obj.Agents{jAgent});
                    end
                               
                end
                     
                
            end
            
            for iAgent = 1:obj.nAgents
obj.Agents{iAgent} = obj.Agents{iAgent}.timestep();
            end
        end
        
        function [isAgentStatic, collision, goalsReached] = assessagents(obj)
            collision = zeros(obj.nAgents,obj.nAgents);
            goalsReached = 0;
            isAgentStatic = zeros(obj.nAgents,1);
            for iAgent = 1:obj.nAgents
                isAgentStatic(iAgent) = obj.Agents{iAgent}.isAtGoal;
            end
            
            if nnz(isAgentStatic) == obj.nAgents
                goalsReached = 1;
            end
            
            r = 1.06*AgentConstants.RADIUS;
            for iAgent = 1:obj.nAgents
                agent1Pos = obj.Agents{iAgent}.position;
                for jAgent = 1:obj.nAgents
                    agent2Pos = obj.Agents{jAgent}.position;
                    if iAgent ~= jAgent
                        [x,~] = circcirc(agent1Pos(1),agent1Pos(2),r,agent2Pos(1),agent2Pos(2),r);
                        if ~isnan(x)
                            collision(iAgent,jAgent) = 1;
                        end
                    end
                end
            end
            
        end
        
        function [obj] = assessq(obj)
            
            reward = zeros(1,obj.nAgents);
            if nnz(obj.collision) > 0
                
                for iAgent = 1:obj.nAgents
                    if nnz(obj.collision(:,iAgent)) > 0 && ~obj.Agents{iAgent}.isAtGoal
                        reward(iAgent) = RLConstants.COLLISION_PENALTY;
                        
                    else
                        reward(iAgent) = 0;
                    end
                end
                retrocausality = 1;
                
            elseif obj.goalsReached
                
                reward = ones(1,obj.nAgents)*RLConstants.GOAL_REWARD;
                % timing consideration
                efficiency = 1-((obj.time-obj.tOptimal)/obj.tOptimal);
                reward = reward*efficiency;
                
                
                retrocausality = 1;
            else
                for iAgent = 1:obj.nAgents
                    reward(iAgent) = obj.calcreward(obj.Agents{iAgent});
                    retrocausality = 0;
                end
            end
            
            if ~isempty(reward)
                obj = obj.applyreward(reward,retrocausality);
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
            
            % Calculate close to goal heading reward
            if ~isempty(Agent.goalHeading)
                maxDeltaHeading = pi/2;
                deltaHeading = abs(wrapToPi(Agent.goalHeading - Agent.heading));
                if deltaHeading < pi/3
                    deltaHeading = 0;
                end
                shR = (0.5 - (deltaHeading/maxDeltaHeading)) * RLConstants.SIMILAR_HEADING_WEIGHT;
                R = R + shR;
            end
            
        end
        
        function obj = applyreward(obj,reward,retrocausality)
            global Q
            global S
            global visitCount
            
            % Needs to take an average of all the rewards for that action
            % in that state
            for iAgent = 1:obj.nAgents
                Agent = obj.Agents{iAgent};
                
                if retrocausality
                    
                    nonGoalIdx = Agent.PastStates ~= 0;
                    nonGoalStates = Agent.PastStates(nonGoalIdx);
                    nonGoalActions = Agent.PastActions(nonGoalIdx);
                                     
                    if ~isempty(nonGoalActions)
                        for i = 1:length(nonGoalActions)
                            iAction = length(nonGoalActions) - i + 1;
                            adjustedLearningRate = RLConstants.LEARNING_RATE^(0.9+(iAction/10));
                            iState = iAction;
                            actionId = nonGoalActions(iAction);
                            stateId = nonGoalStates(iState);
                            
                            if stateId > 0
                                sQ = Q(:,:,stateId);
                                oldQ = sQ(actionId);
                                
                                surroundingStates = [stateId - 14, stateId - 13,...
                                    stateId - 12, stateId - 1, stateId, stateId + 1,...
                                    stateId + 12, stateId + 13, stateId + 14];
                                isInvalidState = zeros(1,length(surroundingStates));
                                for j = 1:length(surroundingStates)
                                    isInvalidState(j) = surroundingStates(j) > 169 || surroundingStates(j) < 1  || isnan(S{surroundingStates(j)}(1));
                                end
                                
                                surroundingStates(isInvalidState == 1) = [];
                                maxQ = zeros(1,length(surroundingStates));
                                for j = 1:length(surroundingStates)
                                    maxQ(i) = max(max(Q(:,:,surroundingStates(j))));
                                end
                                
                                meanMaxQ = mean(maxQ);
                                
                                newQ = ((1 - adjustedLearningRate) * oldQ)...
                                    + adjustedLearningRate * (reward(iAgent)...
                                    + RLConstants.DISCOUNT_FACTOR * meanMaxQ);
                                
                                sQ(actionId) = newQ;
                                Q(:,:,stateId) = sQ;
                            end
                        end
                    else
                        actionId = Agent.actionId;
                        stateId = Agent.stateId;
                        if stateId > 0
                            sQ = Q(:,:,stateId);
                            oldQ = sQ(actionId);
                            
                            surroundingStates = [stateId - 14, stateId - 13,...
                                stateId - 12, stateId - 1, stateId, stateId + 1,...
                                stateId + 12, stateId + 13, stateId + 14];
                            isInvalidState = zeros(1,length(surroundingStates));
                            for i = 1:length(surroundingStates)
                                isInvalidState(i) = surroundingStates(i) > 169 || isnan(S{surroundingStates(i)}(1));
                            end
                            
                            surroundingStates(isInvalidState == 1) = [];
                            maxQ = zeros(1,length(surroundingStates));
                            for i = 1:length(surroundingStates)
                                maxQ(i) = max(max(Q(:,:,surroundingStates(i))));
                            end
                            
                            meanMaxQ = mean(maxQ);
                            
                            newQ = ((1 - RLConstants.LEARNING_RATE) * oldQ)...
                                + RLConstants.LEARNING_RATE * (reward(iAgent)...
                                + RLConstants.DISCOUNT_FACTOR * meanMaxQ);
                            
                            sQ(actionId) = newQ;
                            Q(:,:,stateId) = sQ;
                        end
                        
                    end
                    
                end
                
            end
            
        end
        
    end
    
end
