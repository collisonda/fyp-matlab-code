classdef RLConstants < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        DISCOUNT_FACTOR = 0.5;
        INITIAL_EPSILON = 0.7;
        EPSILON_DECAY_RATE = 0.98;
        GOAL_DISTANCE_WEIGHT = 60;
        SIMILAR_HEADING_WEIGHT = 10;
        GOAL_REWARD = 100;
        COLLISION_PENALTY = -200;
        BASE_FACTOR = 0.05;
    end

end

