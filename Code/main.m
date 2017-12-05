% Defines the scenario you want to run, invokes functions to solve the problem and plot the solution.
clear; clc; 
% close all;

%% Add paths
addpath(genpath(fullfile(fileparts(mfilename('fullpath')), 'Utils')));

%% Load scenario params
LoadScenario;

%% Task definition

task                 = struct;
task.N               = numSamples;                                          % Number of samples

%% Road Definition

task.road            = roadsegment;
task.road.speedlimit = speedLimit;                                          % [m/s] speed limit
task.oncomingveh     = false;                                               % is there an oncoming vehicle
task.adjacentveh     = isAdjVeh1;                                           % is there a vehicle on the adjacent lane
task.adjacentveh2    = isAdjVeh2;                                           % is there a 2:nd vehicle on the adjacent lane
task.twonorm         = true;                                                % using the two-norm as stopping criteria, 0 uses infinity norm
task.gradient        = false;                                               % Decide whether the gradient term should be povertakeent in the sequential QP. (seems to work better without this, so probably you can keep it false at all times)
task.ego             = egoVehiclePosition;                                  % should the ego vehicle be 1:st, 2:nd ord 3:rd

%% Ego vehicle

task.E               = standardcar;
task.E.vref          = task.road.speedlimit*ones(task.N,1);                 % [m/s] reference velocity
task.E.horizon       = horizon;                                             % [m] Length of prediction horizon
task.E.vxmax         = task.road.speedlimit;
task.E.x0            = 0;                                                   % [m] initial longitudinal position
task.E.vx0           = task.E.vref(1);                                      % [m/s] initial longitudinal speed
task.E.y0            = task.road.lanewidth/2;                               % [m] initial lateral position
task.E.vy0           = 0;                                                   % [m/s] Initial lateral speed
task.E.Fr            = 0;                                                   % [N] Rolling overtakeistance

%% Leading vehicle

task.L               = standardcar;
task.L.x0            = posLeading;                                          % [m] initial longitudinal position
task.L.vx            = speedLeading;                                        % [m/s] speed
task.L.y0            = task.road.lanewidth/2;                               % [m]   initial lateral position

%% Vehicles on an adjacent lane

if task.adjacentveh
    task.A           = standardcar;
    task.A.vx        = speedAdjVeh1;                                        % [m/s] speed
    task.A.x0        = posAdjVeh1;                                          % [m] initial longitudinal position
    task.A.y0        = 1.5*task.road.lanewidth;                                 % [m] initial lateral position
end

if task.adjacentveh2
    task.A2          = standardcar;
    task.A2.vx       = speedAdjVeh2;                                        % [m/s] speed
    task.A2.x0       = posAdjVeh2;                                          % [m] initial longitudinal position
    task.A2.y0       = 1.5*task.road.lanewidth;                                 % [m] initial lateral position
end

%% Model Predictive Control (MPC)

egoPos = [];
robustness = 0;

for i=1:numSteps
    init;
    
    overtake   = solveOvertake(task);                                       % Optimal overtaking maneuver
    if ~overtake.status.problem
        endTime = overtake.t(end);
    else
        endTime = 30;
    end
    task.dt    = endTime/task.N;
    task.t     = (0:task.dt:endTime)';
    follow     = solveFollow(task);                                         % Optimal following maneuver
    
    % To overtake or not to overtake    
    overtakeCost = realmax('single');
    followCost   = realmax('single');
    
    if overtake.status.problem && follow.status.problem
        error('No feasible solutions for both follow and overtake');
    end
    
    if ~overtake.status.problem
        overtakeCost = sum((overtake.vEx-task.E.vref).^2)*w(1) +...
            sum((overtake.yE-task.E.yref).^2)*w(2) + ...
            sum(overtake.vEy.^2)*w(3) +...
            sum(overtake.Ax.^2)*w(4) + sum(overtake.Ay.^2)*w(5) +...
            sum(overtake.Jx.^2)*w(6);
    else
        warning('Overtake infeasible');
    end

    if ~follow.status.problem
        followCost   = sum((follow.vEx-task.E.vref).^2)*w(1) +...
            sum((follow.yE-task.E.yref).^2)*w(2) + ...
            sum(follow.vEy.^2)*w(3) +...
            sum(follow.Ax.^2)*w(4) + sum(follow.Ay.^2)*w(5) +...
            sum(follow.Jx.^2)*w(6);
    else
        warning('Following infeasible');
    end
        
    if followCost - overtakeCost > eps(followCost)
        robustness = robustness+1;
        if robustness >= 5
            decision = overtake;
        else
            decision = follow;
        end
    else        
        decision = follow;
        robustness = 0;
    end
    
    % Update scenario for next time step
    task = UpdateScenario(task, decision);
end

animate_res;

%% Remove added paths
rmpath(genpath(fullfile(fileparts(mfilename('fullpath')), 'Utils')));