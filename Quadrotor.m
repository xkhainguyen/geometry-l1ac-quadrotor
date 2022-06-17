classdef Quadrotor
    %QUADROTOR Summary of this class goes here
    %   This is for fix step
    
    properties
        params = struct('m',0.075,...
            'd',0.315,...
            'g',9.81,...
            'ctf',8.004*10e-4,...
            'J',diag([5.8*10e-5,7.2*10e-5,10*10e-5]),...
            'T',[1,1,1,1;0,-0.315,0,0.315;0.315,0,-0.315,0;-0.008004,0.008004,-0.008004,0.008004])
%         states0 = [zeros(6,1); reshape(eye(3,3),[9,1]); zeros(3,1)]   % [p (m); v (m/s); reshape(R,[9,1]); W (rad/s)] 
%         dstates0 = zeros(18,1)
        states = [zeros(6,1); reshape(eye(3,3),[9,1]); zeros(3,1)]
        dstates = [zeros(6,1); reshape(eye(3,3),[9,1]); zeros(3,1)]
        statesAll
        dstatesAll 
    end
   	properties (Dependent)
        B
        Bp
    end
    methods
        function out = get.B(obj)
            B11 = -obj.params.m\reshape(obj.states(7:15),[3,3])*[0;0;1];
            B12 = zeros(3, 3);
            B21 = zeros(3, 1);
            B22 = inv(obj.params.J);
            out = [B11 B12; B21 B22];
        end
        
        function out = get.Bp(obj)
            B11 = obj.params.m\reshape(obj.states(7:15),[3,3])*[1;0;0];
            B12 = obj.params.m\reshape(obj.states(7:15),[3,3])*[0;1;0];
            B21 = zeros(3, 1);
            B22 = zeros(3, 1);
            out = [B11 B12; B21 B22];
        end
        
        function obj = Quadrotor(quad)
            %CARTPOLEPARAMS Construct an instance of this class
            %   Detailed explanation goes here
            if nargin > 0
                obj.params.m = quad.m;
                obj.params.d = quad.d;
                obj.params.g = quad.g;
                obj.params.ctf = quad.ctf;
                obj.params.T = [1,1,1,1;0,-quad.d,0,quad.d;quad.d,0,-quad.d,0;...
                    -quad.ctf,quad.ctf,-quad.ctf,quad.ctf];
                obj.states(7:15) = reshape(quad.R0,[9,1]); 
            end
        end
              
        function [X, obj] = dynamics(obj, u, sig, geo, params)
            %DYNAMICS Summary of this method goes here
            %   Detailed explanation goes here
            dt = params.dt;
            g = obj.params.g;
            e3 = geo.iFrame.e3;
            m = obj.params.m;
            J = obj.params.J;
            obj.statesAll = [obj.statesAll, obj.states];
            obj.dstatesAll = [obj.dstatesAll, obj.dstates];
%             p = obj.states(1:3);
            v = obj.states(4:6);
            R = reshape(obj.states(7:15),[3,3]);
            W = obj.states(16:18);
            
%             u = [10;1;-1;1];
            
            f = u(1);
            M = u(2:4);
            
            % Equations of motion
            dp = v;
            dR = R*geo.hatMap(W);
            dv = g*e3 - (f/m)*R*e3*sig.b;        
            
            dW = J\(-cross(W,J*W) + M*sig.b);  
            
            % Disturbance
            dz = [dv; dW];
            dz = dz + obj.B*sig.m + obj.Bp*sig.um;
            dv = dz(1:3);
            dW = dz(4:6);
            
            dX = [dp; dv; reshape(dR,[9,1]); dW];
            obj.dstates = dX;
            
            X = obj.states + dt*dX; 
            obj.states = X;
            
        end    
    end
end

