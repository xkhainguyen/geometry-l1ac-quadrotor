classdef L1AC
    %L1AC Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        wc_f = 10   
        wc_M = [4; 6]
        As = -diag([1 1 1 2 2 2])*5
        z_hat = zeros(6, 1)
        z_hatAll
        ztAll
        sig_hatAll
        uadAll
        uad_prev = [0; 0; 0; 0]
    end
    
    methods            
        function [uad, obj] = calcAd(obj, quad, ub, geo, params)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            Ts = params.Ts;
            obj.z_hatAll = [obj.z_hatAll, obj.z_hat];
            z = [quad.states(4:6); quad.states(16:18)];
            z_tilde = obj.z_hat - z;
            obj.ztAll = [obj.ztAll, z_tilde];
            g = quad.params.g;
            J = quad.params.J;
            m = quad.params.m;
            e3 = geo.iFrame.e3;
            
            % Estimator
            Bb = [quad.B, quad.Bp];
            muy = expm(obj.As*Ts)*z_tilde;
            Phi = obj.As\(expm(obj.As*Ts) - eye(6));
            sigma_hat = -Bb\inv(Phi)*muy;
            sig_mHat = sigma_hat(1:4);
            sig_umHat = sigma_hat(5:6);
            obj.sig_hatAll = [obj.sig_hatAll, sig_mHat];
            
            % Filter
            uad = zeros(4, 1); % init column vector
            uad(1) = (Ts*obj.wc_f)*sig_mHat(1) + (1-Ts*obj.wc_f)*obj.uad_prev(1);
            uad(2) = (Ts*obj.wc_M(1))*sig_mHat(2) + (1-Ts*obj.wc_M(1))*obj.uad_prev(2);
            uad(3) = (Ts*obj.wc_M(2))*sig_mHat(3) + (1-Ts*obj.wc_M(2))*obj.uad_prev(3);
            obj.uad_prev = uad;
            uad = -uad; 
            obj.uadAll = [obj.uadAll, uad];
            
            u = ub + uad; 
                
            f = u(1);
            M = u(2:4);
            
            R = reshape(quad.states(7:15),[3,3]);
            W = quad.states(16:18);
            
            % Predictor
            dv_hat = g*e3 - (f/m)*R*e3;         
            dW_hat = J\(-cross(W,J*W) + M);  
            
            dz_hat = [dv_hat; dW_hat];
            dz_hat = dz_hat + quad.B*sig_mHat + quad.Bp*sig_umHat + obj.As*z_tilde;
            
            obj.z_hat = obj.z_hat + Ts*dz_hat; 
            
        end
    end
end

