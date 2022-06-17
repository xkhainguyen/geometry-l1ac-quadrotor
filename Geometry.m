classdef Geometry
    %GEOMETRY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        iFrame = struct('e1',[1;0;0],'e2',[0;1;0],'e3',[0;0;1])
    end
    
    methods
%         function obj = Geometry(inputArg1,inputArg2)
%             %GEOMETRY Construct an instance of this class
%             %   Detailed explanation goes here
%             obj.Property1 = inputArg1 + inputArg2;
%         end
        
        function out = hatMap(~,x)
            out = [0,-x(3),x(2);x(3),0,-x(1);-x(2),x(1),0];
        end
        
        function out = veeMap(~,x)
            out = [x(3,2);x(1,3);x(2,1)];
        end
    end
end

