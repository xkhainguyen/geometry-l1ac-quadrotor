%% GEOMETRIC CONTROL WITH L1 ADAPTIVE AUGMENTATION - NAIRA GROUP - SHENG TEAM
% SIMULATION BY XUAN KHAI NGUYEN
tic
clc
close all;
clear all;
% 
L1AC_flag = 0;          % Turn L1AC on/off
params.dt = 0.0005;      % plant interval
params.Ts = 0.002;       % adaptive sampling time
params.Tf = 10;         % simulation timeW
params.t = 0:params.dt:params.Tf;
params.ts = 0:params.Ts:params.Tf;
% Define classes
pln = Planner(1, 3);    % scenarios
geo = Geometry;
quad = Quadrotor;
quad.states = initStates(pln);
ctrl = Controller;
l1ac = L1AC;
vs = Visualize;
disp("Setup done!");
%% MAIN LOOPS
%
for i=1:length(params.t) % Real time
    [sig, pln] = pln.calcDist(quad, params.t(i));  % Generating disturbances
    sCond = ~mod(i-1,round(params.Ts/params.dt)); % Condition to sample
    if (sCond) % Sampling
        [traj, pln] = pln.calcTraj(params.t(i));   % Generating trajectories
        [ub, ctrl] = ctrl.calcInput(quad, traj, geo, params);   % Geometric control
        %         ub = [0.73575;0;0;0];
        % 0.73575
        if (L1AC_flag)
            % L1AC on
            [uad, l1ac] = l1ac.calcAd(quad, ub, geo, params);   
        else
            % L1AC off
            uad = [0;0;0;0];
        end
        u = ub + uad;
    end
    [~, quad] = quad.dynamics(u, sig, geo, params); % Quadrotor dynamics
end
disp("Main loop done!");
%% VISUALIZE THE RESULTS
%
vs.plot(quad, pln, ctrl, l1ac, params);
% vs.animate(quad, pln, geo);

% figure;
% plot(ts,l1ac.ztAll(3,:));
disp('Simulation finished');
toc