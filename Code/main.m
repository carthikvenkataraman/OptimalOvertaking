% Defines the scenario you want to run, invokes functions to solve the problem and plot the solution.
clear; clc; close all;

%% Add paths
addpath(genpath(fullfile(fileparts(mfilename('fullpath')), 'YALMIP-master')));

%% Load scenario params
LoadScenario;

%% Task definition

task                 = struct;
task.N               = numSamples;                             % Number of samples

%% Road Definition

task.road            = roadsegment;
task.road.speedlimit = speedLimit;                             % [m/s] speed limit
task.oncomingveh     = false;                                  % is there an oncoming vehicle
task.adjacentveh     = isAdjVeh1;                              % is there a vehicle on the adjacent lane
task.adjacentveh2    = isAdjVeh2;                              % is there a 2:nd vehicle on the adjacent lane
task.twonorm         = true;                                   % using the two-norm as stopping criteria, 0 uses infinity norm
task.gradient        = false;                                  % Decide whether the gradient term should be povertakeent in the sequential QP. (seems to work better without this, so probably you can keep it false at all times)
task.ego             = egoVehiclePosition;                     % should the ego vehicle be 1:st, 2:nd ord 3:rd

%% Ego vehicle

task.E               = standardcar;
task.E.vref          = task.road.speedlimit*ones(task.N,1);    % [m/s] reference velocity
task.E.horizon       = horizon;                                % [m] Length of prediction horizon
task.E.vxmax         = task.road.speedlimit;
task.E.x0            = 0;                                      % [m] initial longitudinal position
task.E.vx0           = task.E.vref(1);                         % [m/s] initial longitudinal speed
task.E.y0            = task.road.lanewidth/2;                  % [m] initial lateral position
task.E.vy0           = 0;                                      % [m/s] Initial lateral speed
task.E.Fr            = 0;                                      % [N] Rolling overtakeistance

%% Leading vehicle

task.L               = standardcar;
task.L.x0            = posLeading;                             % [m]   initial longitudinal position
task.L.vx            = speedLeading;                           % [m/s] speed
task.L.y0            = task.road.lanewidth/2;                  % [m]   initial lateral position

%% Vehicles on an adjacent lane

if task.adjacentveh
    task.A           = standardcar;
    task.A.vx        = speedAdjVeh1;                          % [m/s] speed
    task.A.x0        = posAdjVeh1;                            % [m] initial longitudinal position
    task.A.y0        = task.road.lanewidth;                   % [m] initial lateral position
end

if task.adjacentveh2
    task.A2          = standardcar;
    task.A2.vx       = speedAdjVeh2;                          % [m/s] speed
    task.A2.x0       = posAdjVeh2;                            % [m] initial longitudinal position
    task.A2.y0       = task.road.lanewidth;                   % [m] initial lateral position
end

%% Model Predictive Control (MPC)

egoPos = [];

for i=1:numSteps
    init;
    
    overtake   = solveOvertake(task);                          % Optimal overtaking maneuver
    task.dt    = overtake.t(end)/task.N;
    task.t     = (0:task.dt:overtake.t(end))';
    follow     = solveFollow(task);                            % Optimal following maneuver
    
    % Calculate accelerations needed for plotting
    Ax=diff(overtake.vEx)./diff(overtake.t); Ax=[Ax; Ax(end)];
    Ay=diff(overtake.vEy)./diff(overtake.t); Ay=[Ay; Ay(end)];    
    Ax_follow=diff(follow.vEx)./diff(follow.t); Ax_follow=[Ax_follow; Ax_follow(end)];
    Ay_follow=diff(follow.vEy)./diff(follow.t); Ay_follow=[Ay_follow; Ay_follow(end)];
    
    % Display optimisation status
    fprintf('%s: cost=%1.4f, vx~[%1.0f, %1.0f]km/h, ax~[%1.1f, %1.1f]m/s2, vy~[%1.1f, %1.1f]m/s, ay~[%1.1f, %1.1f]m/s^2, t=%1.2f ms\n', ...
        overtake.status.info,overtake.cost.total, min(overtake.vEx)*3.6, max(overtake.vEx)*3.6, ...
        min(Ax), max(Ax), min(overtake.vEy), max(overtake.vEy), min(Ay), max(Ay),overtake.status.solvertime(end)*1000);
    
    % Plot the solution
    if i == 1 || i ==20 || i==40 || i==60
        plotting(task, overtake, Ax, Ay)
        if i==1
            try_1 = overtake;
            y_traj = overtake.yE;
        else
            try_4 = overtake;
        end
    end
    plotting(task, follow, Ax_follow, Ay_follow)    
    
    % Ego vehicle update
    task.E.vy0 = overtake.vEy(2);
    task.E.vx0 = overtake.vEx(2);
    task.E.y0  = overtake.yE(2);                        % task.E.y0 + overtake.vEy(2)*overtake.ts(1);    
    % task.E.vref    = task.E.vref(2:end);              % ref
    % task.N         = task.N-1;
    % task.E.horizon = task.E.horizon-1;                % [m] Length of prediction horizon                                 %[m] Length of prediction horizon
    
    % Leading vehicle update
    task.L.x0 = task.L.x0 + overtake.ts(1)*task.L.vx - (overtake.vEx(2)*overtake.ts(1)) ;
    % Adjacent vehicle update
    if task.adjacentveh
        task.A.x0 = task.A.x0 + overtake.ts(1)*task.A.vx - (overtake.vEx(2)*overtake.ts(1));
    end
    % 2nd Adjacent vehicle update
    if task.adjacentveh2
        task.A2.x0 = task.A2.x0 + overtake.ts(1)*task.A2.vx - (overtake.vEx(2)*overtake.ts(1));
    end
end

%% Backup

% if i == 1 || i==20
%
%     plotting(task, overtake, Ax, Ay)
%     if i==1
%         try_1 = overtake;
%         y_traj = overtake.yE;
%     else
%         try_4 = overtake;
%     end
% end
% % Test (remove)
% egoPos(end+1) = task.E.y0;
% % Update
% task.E.vref=task.E.vref(2:end); % ref
% task.E.vy0 = overtake.vEy(2);
% task.E.vx0 = overtake.vEx(2);
% % Troubleshooting y-axis for host
%
% task.E.y0 = y_traj(i);
% task.N=task.N-1;
% task.E.horizon=task.E.horizon-1;  %[m] Length of prediction horizon                                 %[m] Length of prediction horizon
%
% % Leading veh update
% task.L.x0 = task.L.x0 - overtake.xE(2);
% %task.L.y0 = task.L.y0 - overtake.yE(2); %If host-vehicle fixed in origo
% % Adjacent veh update
% task.A.x0 = task.A.x0 - overtake.xE(2);
% %task.A.y0 = task.A.y0 - overtake.yE(2); %If host-vehicle fixed in origo

%% Remove added paths
rmpath(genpath(fullfile(fileparts(mfilename('fullpath')), 'YALMIP-master')));