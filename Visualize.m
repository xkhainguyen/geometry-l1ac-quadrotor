classdef Visualize
    %VISUALIZE Draw figures for visualization
    %   Compatible with MainGeoL1_v1
    
    properties
        
    end
    
    methods
        function out = plot(~, quad, pln, ctrl, l1ac, params)
            ts = params.ts;
            states = zeros(3, length(ts));
            sig_mAll = zeros(4, length(ts));
            for i = 1:size(quad.statesAll, 1)
                states(i,:) = downsample(quad.statesAll(i,:), round(params.Ts/params.dt));
            end
            for i = 1:size(sig_mAll, 1)
                sig_mAll(i,:) = downsample(pln.sig_mAll(i,:), round(params.Ts/params.dt));
            end
            
%             figure(1)   % position
%             name = ['x','y','z'];
%             for i=1:3
%                 subplot(3,1,i);
%                 plot(t,quad.statesAll(i,:),'b')
%                 hold on; 
%                 plot(ts, pln.pdAll(i,:),'--r');  
%                 title(name(i),'Interpreter','latex');
% %                 ylim([-0.1 0.1]); % Limit for Case 0
%                 ylim([-1 1]);       % Limit for other cases
%             end   
            
            figure(1)   % position error
            name = ["$e_x$","$e_y$","$e_z$"];
            for i=1:3
                subplot(3,1,i);
                plot(ts,pln.pdAll(i,:) - states(i,:),'k','LineWidth',1)
                title(name(i),'Interpreter','latex');
%                 ylim([-0.1 0.1]); % Limit for Case 0
                ylim([-1 1]);       % Limit for other cases
            end  
            
            figure(2)   % attitude
            plot(ts,ctrl.PsiAll,'k','LineWidth',1);
            title('Attitude error $\Psi$','Interpreter','latex');
            ylim([-0.5 2.5]);
            
%             figure(3)   % angular velocity
%             name = ["$\Omega_x$","$\Omega_y$","$\Omega_z$"];
%             for i=1:3
%                 subplot(3,1,i);
%                 plot(ts,states((i+15), :),'b');
%                 hold on;
%                 plot(ts, ctrl.WdAll(i, :),'--r');   
%                 title(name(i),'Interpreter','latex');
%             end      
            
            figure(3)   % angular velocity error
            name = ["$e_{\Omega_x}$","$e_{\Omega_y}$","$e_{\Omega_z}$"];
            for i=1:3
                subplot(3,1,i);
                plot(ts,ctrl.WdAll(i, :) - states((i+15), :),'k','LineWidth',1);
                title(name(i),'Interpreter','latex');
            end      
           
            figure(4)   % control input
            for i=1:4
                subplot(4,1,i);
                plot(ts,ctrl.inputAll(i,:),'k','LineWidth',1);
%                 ylim([-200 200]);
                title(['Control input ', num2str(i)]);
            end

            try
            figure(5);
            plot(ts,sig_mAll-l1ac.sig_hatAll,'LineWidth',1);
%             label1 = "$\sigma$";
%             label2 = "$\hat{\sigma}$";
%             legend(label1,label2,'Interpreter','latex')
            title("Disturbance Estimation");
            catch
            end
                
            out = 1;
        end
        
        function out = animate(~,quad, pln, geo)
            %METHOD1 Create animation for quadrotors
            %   Detailed explanation goes here
            
            X = interp1((1:size(quad.statesAll,2))', quad.statesAll',...
                (1:round(size(quad.statesAll,2)/300):size(quad.statesAll,2))')'; % Fewer points, faster run
            m = size(X,2);
            d = quad.params.d;
            e1 = geo.iFrame.e1;
            e2 = geo.iFrame.e2;
            pd = interp1((1:size(pln.pdAll,2))', pln.pdAll',...
                (1:round(size(quad.statesAll,2)/300):size(quad.statesAll,2))')';
 
            figure
            for i=1:m
                p = X(1:3,i);
                R = reshape(X(7:15, i),[3,3]);
                
                r1 = (p + d*R*e1);  % rotor position
                r2 = (p - d*R*e1);
                r3 = (p + d*R*e2);
                r4 = (p - d*R*e2);
                
                plot3(X(1,:),X(2,:),X(3,:),'Color','#0072BD','LineWidth',0.25);
                hold on;
                plot3(pd(1,:),pd(2,:),pd(3,:),'--','Color','#A2142F','LineWidth',0.75);
                
                hold on;
                plot3([p(1);r1(1)],[p(2);r1(2)],[p(3);r1(3)],'-k','LineWidth',1.5);
                hold on;
                plot3([p(1);r2(1)],[p(2);r2(2)],[p(3);r2(3)],'-k','LineWidth',1.5);
                hold on;
                plot3([p(1);r3(1)],[p(2);r3(2)],[p(3);r3(3)],'-k','LineWidth',1.5);
                hold on;
                plot3([p(1);r4(1)],[p(2);r4(2)],[p(3);r4(3)],'-k','LineWidth',1.5);
                hold on;
                plot3(r1(1),r1(2),r1(3),'o','Color','#0072BD','MarkerSize',5,'MarkerFaceColor','#0072BD');
                hold on;
                plot3(r2(1),r2(2),r2(3),'o','Color','#0072BD','MarkerSize',5,'MarkerFaceColor','#0072BD');
                hold on;
                plot3(r3(1),r3(2),r3(3),'o','Color','#A2142F','MarkerSize',5,'MarkerFaceColor','#A2142F');
                hold on;
                plot3(r4(1),r4(2),r4(3),'o','Color','#A2142F','MarkerSize',5,'MarkerFaceColor','#A2142F');
                hold on;
                grid on;
                axis([-2 6 -2 2 -2 2]);
                axis on;
                xlabel('x');ylabel('y');zlabel('z');
                drawnow
%                 pause(0.05)
                hold off;
            end
            out = 1;
        end
    end
end

