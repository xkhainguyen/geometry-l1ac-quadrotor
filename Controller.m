classdef Controller
    %CONTROLLER Summary of this class goes here
    %   Calculate derivative via delta over time
    
    properties
        % Controller from paper
        kx = diag([6.2*10e-1 0.5*10e-1 0.5*10e-1])
        kv = diag([3*10e-1 0.3*10e-1 0.3*10e-1])
        kr = diag([3*10e-2 1*10e-2 5*10e-2])
        kw = diag([3*10e-3 1.5*10e-3 7*10e-3])
        % My fine-tuned controller
%         kx = diag([7*10e-1 2*10e-1 2*10e-1])
%         kv = diag([6*10e-1 1*10e-1 1*10e-1])
%         kr = diag([3*10e-2 2*10e-2 3*10e-2])
%         kw = diag([3*10e-3 3*10e-3 3*10e-3])
        
        %         Below is parameters for a poorly-tuned geometric controller:
        %         kx = diag([0.6*10e-1 0.1*10e-1 0.1*10e-1])
        %         kv = diag([3*10e-1 0.3*10e-1 0.3*10e-1])
        %         kr = diag([0.3*10e-2 0.1*10e-2 1*10e-2])
        %         kw = diag([3*10e-3 1.5*10e-3 7*10e-3])
        
        dv_prev = zeros(3,1);
        ubAll
        inputAll
        PsiAll
        WdAll
    end
    
    methods
        function obj = Controller(kxx, kvv, krr, kww)
            %CONTROLLER Construct an instance of this class
            %   Detailed explanation goes here
            if nargin > 0
                obj.kx = kxx;
                obj.kv = kvv;
                obj.kr = krr;
                obj.kw = kww;
            end
        end
        
        function [out, obj] = calcInput(obj, quad, traj, geo, params)
            %CALCINPUT Summary of this method goes here
            %   Detailed explanation goes here
            m = quad.params.m;
            g = quad.params.g;
            J = quad.params.J;
            e3 = geo.iFrame.e3;
            X = quad.states;
            Ts = params.Ts;
            
            p = X(1:3);
            v = X(4:6);
            R = reshape(X(7:15),[3,3]);
            W = X(16:18);
            
            ep = p - traj.pd;
            ev = v - traj.vd;
            
            a1 = (-obj.kx*ep - obj.kv*ev - m*g*e3 + m*traj.ddpd);
            
            b3d = -a1/norm(a1); % (14)
            
            f = dot(-a1,R*e3);   % (12)
            
            dv =  g*e3 - (1/m)*f*R*e3;  % (3)
            ea = dv - traj.ddpd;
            
            a2 = cross(b3d,traj.b1d);
            b1d = -(1/norm(a2))*cross(b3d,a2);
            b2d = a2/norm(a2);
            
            Rd = [b1d b2d b3d];
            %%
            da1 = -obj.kx*ev - obj.kv*ea + m*traj.d3pd; % diff a1
            db3d = -da1/norm(a1) + (dot(a1,da1)/norm(a1)^3)*a1; % diff (14)
            da2 = cross(db3d,b1d) + cross(b3d,traj.db1d);    % diff a2
            db2d = da2/norm(a2) - (dot(a2,da2)/norm(a2)^3)*a2;  % diff b2
            db1d = cross(db2d,b3d) + cross(b2d,db3d);   % diff b1d = cross(b2d, b3d)
            
            % more accurate method for d3dx (both methods have almost same results though)
            d3p = (dv - obj.dv_prev)/Ts;
            obj.dv_prev = dv;
            ej = d3p - traj.d3pd;
            
            dda1 = - obj.kx*ea - obj.kv*ej + m*traj.d4pd;   % diff da1
            ddb3d = - dda1/norm(a1) + (2/norm(a1)^3)*dot(a1,da1)*da1 ...
                + ((norm(da1)^2 + dot(a1,dda1))/norm(a1)^3)*a1 - (3/norm(a1)^5)*(dot(a1,da1)^2)*a1; % diff db3d
            dda2 = cross(ddb3d,b1d) + cross(db3d,db1d) ...
                + cross(db3d,db1d) + cross(b3d,traj.ddb1d);   % diff da2
            ddb2d = dda2/norm(a2) - (2/norm(a2)^3)*dot(a2,da2)*da2 ...
                - ((norm(da2)^2 + dot(a2,dda2))/norm(a2)^3)*a2 + (3/norm(a2)^5)*(dot(a2,da2)^2)*a2;
            ddb1d = cross(ddb2d,b3d) + cross(db2d,db3d) ...
                + cross(db2d,db3d) + cross(b2d,ddb3d);
            
            dRd = [db1d db2d db3d];
            ddRd = [ddb1d ddb2d ddb3d];
            Wd = geo.veeMap(Rd'*dRd); % like (4)
            obj.WdAll = [obj.WdAll Wd];
            dWd = geo.veeMap(Rd'*ddRd - geo.hatMap(Wd)*geo.hatMap(Wd)); % diff the previous eq
            %%
            er = 0.5*geo.veeMap(Rd'*R - R'*Rd);  % (10)
            ew = W - R'*Rd*Wd; % (11)
            
            Psi = 0.5*trace(eye(3) - Rd'*R);   % (8)
            obj.PsiAll = [obj.PsiAll, Psi];
            
            M = - obj.kr*er - obj.kw*ew + cross(W,J*W) - J*(geo.hatMap(W)*R'*Rd*Wd - R'*Rd*dWd); % (13)
            
            out = [f; M];
            wIn = quad.params.T\out;
            obj.inputAll = [obj.inputAll, wIn];
            obj.ubAll = [obj.ubAll, out];
        end
    end
end

%% ANOTHER CONTROLLER
% classdef Controller
%     %CONTROLLER Summary of this class goes here
%     %   Detailed explanation goes here
%
%     properties
%         kx = 69.44
%         kv = 24.304
%         kr = 8.81
%         kw = 2.54
%         ubAll
%         inputAll
%         PsiAll
%         WdAll
%     end
%
%     methods
%         function obj = Controller(kxx, kvv, krr, kww)
%             %CONTROLLER Construct an instance of this class
%             %   Detailed explanation goes here
%             if nargin > 0
%                 obj.kx = kxx;
%                 obj.kv = kvv;
%                 obj.kr = krr;
%                 obj.kw = kww;
%             end
%         end
%
%         function [out, obj] = calcInput(obj, quad, traj, geo)
%             %CALCINPUT Summary of this method goes here
%             %   Detailed explanation goes here
%             m = quad.params.m;
%             g = quad.params.g;
%             J = quad.params.J;
%             e3 = geo.iFrame.e3;
%             X = quad.states;
%
%             p = X(1:3);
%             v = X(4:6);
%             R = reshape(X(7:15),[3,3]);
%             W = X(16:18);
%
%             psid = traj.psid;
%             dpsid = traj.dpsid;
%             ddpsid = traj.ddpsid;
%             ddpd = traj.ddpd;
%             dad = traj.d3pd;
%             ddad = traj.d4pd;
%
%             ep = p - traj.pd;
%             ev = v - traj.vd;
%
%             Fd = -obj.kx*ep - obj.kv*ev + m*g*e3 + m*traj.ddpd;
%
%             b3d = Fd/norm(Fd); % (14)
%
%             f = dot(Fd,R*e3);   % (12)
%
%             bint = [cos(psid), sin(psid), 0]';
%             b2d = cross(b3d,bint)/norm(cross(b3d,bint));
%             b1d = cross(b2d, b3d);
%             Rd = [b1d b2d b3d];
%             %%
%             hW = (dad - dot(b3d, dad)*b3d)/norm(ddpd + g*e3);
%             W1d = -dot(hW, b2d);
%             W2d = dot(hW, b1d);
%             W3d = dot(dpsid*e3, b3d);
%             Wd = [W1d W2d W3d]';
% %             hdW = (ddad - (dot(cross(Wd,b3d),ddpd)+ dot(b3d,ddad))*b3d...
% %                 +dot(b3d,dad)*geo.hatMap(W)*b3d)/norm(ddpd + g*e3)...
% %                 -cross(W,b3d)-cross(W,cross(W, b3d));
%             hdW = ((ddad - (dot(cross(Wd,b3d),ddpd)+ dot(b3d,ddad))*b3d...
%                 +dot(b3d,dad)*geo.hatMap(W)*b3d) -cross(W,b3d)-cross(W,cross(W, b3d)))/norm(ddpd + g*e3);
%             dW1d = - dot(hdW, b2d);
%             dW2d = dot(hdW, b1d);
%             dW3d = dot(ddpsid*e3, b3d);
%             obj.WdAll = [obj.WdAll Wd];
%             dWd = [dW1d dW2d dW3d]';
%             %%
%             er = 0.5*geo.veeMap(Rd*R - R*Rd);  % (10)
%             ew = W - R'*Rd*Wd; % (11)
%
%             Psi = 0.5*trace(eye(3) - Rd.'*R);   % (8)
%             obj.PsiAll = [obj.PsiAll, Psi];
%
%             M = - obj.kr*er - obj.kw*ew; % (13)
%
%             out = [f; M];
%             wIn = quad.params.T\out;
%             obj.inputAll = [obj.inputAll, wIn];
%             obj.ubAll = [obj.ubAll, out];
%         end
%     end
% end

