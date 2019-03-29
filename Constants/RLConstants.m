classdef RLConstants < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        DISCOUNT_FACTOR = 0.9;
        INITIAL_EPSILON = 0.5;
        EPSILON_DECAY_RATE = 0.98;
        GOAL_DISTANCE_WEIGHT = 5;
        SIMILAR_HEADING_WEIGHT = 2;
        GOAL_REWARD = 400;
        COLLISION_PENALTY = -100;
        BASE_FACTOR = 0.02;
    end

end

