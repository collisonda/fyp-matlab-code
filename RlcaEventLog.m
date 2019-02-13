classdef RlcaEventLog < handle
    %RLCAEVENTLOG Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        tStart
        tNow
    end
    
    methods
        function obj = RlcaEventLog()
            %RLCAEVENTLOG Construct an instance of this class
            %   Detailed explanation goes here
            obj.tStart = datetime('now');
        end
        
        function obj = createEvent(obj,string)
            obj.tNow = datetime('now');
            tElapsed = duration(obj.tNow - obj.tStart,'Format','mm:ss');
            disp(['[' char(tElapsed) ']: ' string])
        end
    end
end

