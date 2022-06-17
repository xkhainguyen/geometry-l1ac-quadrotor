%% MAKE GIFS FROM FIGURES
% DESCRIPTIVE TEXT
X = interp1((1:size(quad.statesAll,2))', quad.statesAll',...
    (1:round(size(quad.statesAll,2)/300):size(quad.statesAll,2))')'; % Fewer points, faster run
m = size(X,2);
d = quad.params.d;
e1 = geo.iFrame.e1;
e2 = geo.iFrame.e2;
pd = interp1((1:size(pln.pdAll,2))', pln.pdAll',...
    (1:round(size(quad.statesAll,2)/300):size(quad.statesAll,2))')';
figure
%% first frame
i = 1;
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
hold off
gif('myfile.gif');
%%
for i=2:m
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
    gif
end