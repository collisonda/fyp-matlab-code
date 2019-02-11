classdef EnvironmentConstants < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        START_TIME = 0;
        FINISH_TIME = 100;
        TIME_STEP = 0.1;
        X_BOUNDARY = [-100,100];
        Y_BOUNDARY = [-100,100];
        COLOR_ORDER = [0 0.1 1
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

