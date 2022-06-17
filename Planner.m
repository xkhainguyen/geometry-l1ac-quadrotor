classdef Planner
    %PLANNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        pdAll
        sig_mAll
        ex = 0;
        exCase = 1;
    end
    
    methods
        function obj = Planner(v1, v2)
            %PLANNER Construct an instance of this class
            %   Detailed explanation goes here
            if nargin > 0
                obj.ex = v1;
                obj.exCase = v2;
            end
        end
        function states0 = initStates(obj) 
            switch obj.ex
                case 0
                    states0 = [zeros(6,1); reshape(eye(3,3),[9,1]); zeros(3,1)];
                case 1
                    states0 = [0 ; 0; -1; zeros(3,1); reshape(eye(3,3),[9,1]); zeros(3,1)];   % [p (m); v (m/s); reshape(R,[9,1]); W (rad/s)] 
                case 2
                    states0 = [zeros(6,1); reshape([1,0,0;0,-0.9995,-0.0314;0,0.0314,-0.9995],[9,1]); zeros(3,1)];     
                otherwise
                    error('No existing scenario');
            end
        end
        function [traj, obj] = calcTraj(obj, t)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            switch obj.ex
                case 0
                    traj.pd = [0; 0; 0];
                    traj.vd = [0; 0; 0];
                    traj.ddpd = [0; 0; 0];
                    traj.d3pd = [0; 0; 0];
                    traj.d4pd = [0; 0; 0];
                    traj.b1d = [cos(0); sin(0); 0];
                    traj.db1d = [0; 0; 0];
                    traj.ddb1d = [0; 0; 0];
                    
                case 1
                    traj.pd = [0.4*t; 0.4*sin(pi*t); -0.6*cos(pi*t)];
                    traj.vd = [0.4; 0.4*pi*cos(pi*t); 0.6*pi*sin(pi*t)];
                    traj.ddpd = [0; -0.4*pi*pi*sin(pi*t); 0.6*pi*pi*cos(pi*t)];
                    traj.d3pd = [0;-0.4*pi*pi*pi*cos(pi*t); -0.6*pi*pi*pi*sin(pi*t)];
                    traj.d4pd = [0; 0.4*pi*pi*pi*pi*sin(pi*t); -0.6*pi*pi*pi*pi*cos(pi*t)];
                    traj.b1d = [cos(pi*t); sin(pi*t); 0];
                    traj.db1d = [-pi*sin(pi*t); pi*cos(pi*t); 0];
                    traj.ddb1d = [-pi*pi*cos(pi*t); -pi*pi*sin(pi*t); 0];
%                     traj.psid = pi*t;
%                     traj.dpsid = pi;
%                     traj.ddpsid = 0;                  
                case 2
                    traj.pd = [0;0;0];
                    traj.vd = [0;0;0];
                    traj.ddpd = [0;0;0];
                    traj.d3pd = [0;0;0];
                    traj.d4pd = [0;0;0];
                    traj.b1d = [1;0;0];
                    traj.db1d = [0;0;0];
                    traj.ddb1d = [0;0;0];
                   
                    
                otherwise
                    error('No existing scenario');
            end
            
            obj.pdAll = [obj.pdAll, traj.pd];
        end
        
        function [sig, obj] = calcDist(obj, quad, t)
            p1 = quad.states(1);
            sig.um = zeros(2,1);
            sig.m = zeros(4,1); 
            sig.b = 1;
            switch obj.ex
                case 0
                    sig.m(2) = heaviside(t - 2)*((0.2*sin(1*(t - 2)) + 0.01*(t - 2) + 0.15*sin(1.5*(t - 2)))); 
                case 1
                    switch obj.exCase
                        case 1
                        case 2  % State and time-dependent disturbance
                            sig.m(2) = heaviside(t - 2)*0.01*sin(0.75*(t - 2));
                            sig.m(3) = heaviside(t - 2)*0.01*sin(t - 2)*p1^2;
                        case 3
%                             sig.m(1) = 0.2*sin(0.5*(t - 2)) + 0.15*sin(0.75*(t - 2));
                            sig.m(2) = heaviside(t - 2)*0.2*sin(1.5*(t - 2));
                            sig.m(3) = heaviside(t - 2)*(0.1*sin(t - 2) + 0.1*sin(2*(t - 2)));
                        case 4
                            sig.m = zeros(4,1); 
                            if (2 < t)&&(t < 7)
                                sig.b = 1 + 0.4*sin(t-5);
                            end
                        otherwise
                            error('No existing scenario');   
                    end
                case 2
                    switch obj.exCase
                        case 1
                        case 2
                            sig.m(1) = 0;
                            sig.m(2) = 0.001*sin(0.75*(t - 2));
                            sig.m(3) = 0.01*sin(t - 2)*p1^2;
                            sig.m(4) = 0;
                            if (5 < t)&&(t < 22)
                                sig.b = 1 + 0.4*sin(t-5);
%                                 sig.b = 0.5;
                            end
                        case 3
                            sig.m(1) = 0.2*sin(0.5*(t - 2)) + 0.15*sin(0.75*(t - 2));
                            sig.m(2) = 0.001*sin(0.75*(t - 2));
                            sig.m(3) = 0.0008*sin(t - 2) + sin(0.5*(t - 2));
                            sig.m(4) = 0;
                            if (5 < t)&&(t < 22)
                                sig.b = 1 + 0.4*sin(t - 2);
%                                 sig.b = 0.5;
                            end
                        case 4
                            sig.m = zeros(4,1); 
                            if (3 < t)&&(t < 7)
                                sig.b = 1 + 0.4*sin(t - 2);
%                                 sig.b = 0.5;
                            end
                        otherwise
                            error('No existing scenario');
                    end 
%                 
                otherwise
                        error('No existing scenario');    
            end
            obj.sig_mAll = [obj.sig_mAll, sig.m];
                
        end
    end
end

