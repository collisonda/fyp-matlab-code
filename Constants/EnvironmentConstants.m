classdef EnvironmentConstants < handle
    %EnvironmentConstants Class storing all the constants for the RLCA Environment.
    %   Author: Dale Collison
    
    %% EnvironmentConstants - Properties
    properties (Constant)
        START_TIME = 0;             % Simulation start time.
        MAX_TIME = 50;             % Maximum simulation time.
        TIME_STEP = 0.25;           % Time between updates.
        X_BOUNDARY = [-100,100];    % Environment x-axis lower and upper bounds.
        Y_BOUNDARY = [-100,100];    % Environment y-axis lower and upper bounds.
    end

end

