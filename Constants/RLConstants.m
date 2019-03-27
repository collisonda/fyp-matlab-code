classdef RLConstants < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        DISCOUNT_FACTOR = 0.9;
        PREVIOUS_ACTION_DISCOUNT = 0.9;
        INITIAL_EPSILON = 0.5;
        EPSILON_DECAY_RATE = 0.98;
    end

end

