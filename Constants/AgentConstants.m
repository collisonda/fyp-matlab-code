classdef AgentConstants < handle
    %AgentConstants Class storing all the constants for RLCA Agents.
    %   Author: Dale Collison
    
    %% AgentConstants - Properties
    properties (Constant)
        RADIUS = 8;                 % The radius of an agent.
        NEIGHBOURHOOD_RADIUS = 60;  % The radius of the agent's vision.
        VISION_ANGLE = pi/2;      % The arc angle of the agent's vision.
        GOAL_MARGIN = 1;            % The minimum distance the agent must be 
                                    % from its goal position to be considered to have reached it.
        MAX_SPEED = 6;              % Maximum speed of the agent.
        COLOR_ORDER = [0 0.1 1      % Colour values for each agent.
            1 0 0.2
            0.4 0.9 0
            1 0.4 0
            0.6 0 1
            1 0.3 1
            1 0.8 0
            0.35 0.95 0.75
            0 0.9 1];
    end
    
end
