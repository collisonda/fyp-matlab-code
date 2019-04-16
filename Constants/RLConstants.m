classdef RLConstants < handle
    %RLConstants Class storing all the constants for reinforcement learning.
    %   Author: Dale Collison
    
    properties (Constant)
        DISCOUNT_FACTOR = 0.9;         % Discount factor for each past action.
        GOAL_DISTANCE_WEIGHT = 0;       % Weight of the reward based on the 
                                        % change in distance to goal.
        SIMILAR_HEADING_WEIGHT = 10;    % Weight of the reward based on keeping
                                        % a heading close to the goal heading.
        GOAL_REWARD = 100;               % Base reward for reaching the goal.
        COLLISION_PENALTY = -100;       % Base penalty for colliding.   
        
        LEARNING_RATE = 0.01;
        
    end
    
end
