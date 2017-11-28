% Defines the scenario you want to run, invokes functions to solve the problem and plot the solution.
clear; clc; close all;

%% Task definition

task                 = struct;
task.N               = 181;                                    % Number of samples

%% Road Definition

task.road            = roadsegment;              
task.road.speedlimit = 120/3.6;                                % [m/s] speed limit
task.oncomingveh     = false;                                  % is there an oncoming vehicle
task.adjacentveh     = false;                                  % is there a vehicle on the adjacent lane
task.adjacentveh2    = false;                                  % is there a 2:nd vehicle on the adjacent lane
task.twonorm         = true;                                   % using the two-norm as stopping criteria, 0 uses infinity norm
task.gradient        = false;                                  % Decide whether the gradient term should be present in the sequential QP. (seems to work better without this, so probably you can keep it false at all times)
task.ego             = 3;                                      % should the ego vehicle be 1:st, 2:nd ord 3:rd

%% Ego vehicle

task.E               = standardcar;    
task.E.vref          = task.road.speedlimit*ones(task.N,1);    % [m/s] reference velocity
task.E.horizon       = 150;                                    % [m] Length of prediction horizon
task.E.vxmax         = task.road.speedlimit;                
task.E.x0            = 0;                                      % [m] initial longitudinal position
task.E.vx0           = task.E.vref(1);                         % [m/s] initial longitudinal speed
task.E.y0            = task.road.lanewidth/2;                  % [m] initial lateral position
task.E.vy0           = 0;                                      % [m/s] Initial lateral speed
task.E.Fr            = 0;                                      % [N] Rolling resistance

%% Leading vehicle

task.L               = standardcar;                 
task.L.x0            = 75;                                     % [m]   initial longitudinal position
task.L.vx            = 100/3.6;                                % [m/s] speed
task.L.y0            = task.road.lanewidth/2;                  % [m]   initial lateral position

%% Vehicles on an adjacent lane

if task.adjacentveh
    task.A           = standardcar;
    task.A.vx        = 75/3.6;                                % [m/s] speed
    task.A.x0        = -70;                                   % [m] initial longitudinal position
    task.A.y0        = 1.5*task.road.lanewidth;               % [m] initial lateral position
end

if task.adjacentveh2
    task.A2          = standardcar;
    task.A2.vx       = 75/3.6;                                % [m/s] speed
    task.A2.x0       = -50;                                   % [m] initial longitudinal position
    task.A2.y0       = 1.5*task.road.lanewidth;               % [m] initial lateral position
end
 
%% 

Ego_pos = [];
numSteps = 1;

for i=1:numSteps
    init;
    
    res = yalmipsolve(task);                                % Optimal overtaking maneuver
    task.dt = res.t(end)/task.N;
    task.t = (0:task.dt:res.t(end))';
    res_follow=yalmipsolve_follow(task);

    % status convex problem
    Ax=diff(res.vEx)./diff(res.t); Ax=[Ax; Ax(end)];
    Ay=diff(res.vEy)./diff(res.t); Ay=[Ay; Ay(end)];
    
    Ax_follow=diff(res_follow.vEx)./diff(res_follow.t); Ax_follow=[Ax_follow; Ax_follow(end)];
    Ay_follow=diff(res_follow.vEy)./diff(res_follow.t); Ay_follow=[Ay_follow; Ay_follow(end)];
    %fprintf('%s: cost=%1.4f, vx~[%1.0f, %1.0f]km/h, ax~[%1.1f, %1.1f]m/s2, vy~[%1.1f, %1.1f]m/s, ay~[%1.1f, %1.1f]m/s^2, t=%1.2f ms\n', ...
    %       res.status.info,res.cost.total, min(res.vEx)*3.6, max(res.vEx)*3.6, ...
    %      min(Ax), max(Ax), min(res.vEy), max(res.vEy), min(Ay), max(Ay),res.status.solvertime(end)*1000);
    
    plotting(task,res_follow, Ax_follow, Ay_follow)
    % Plot the solution
    if i == 1 || i ==20 || i==40 || i==60
        plotting(task, res, Ax, Ay)
        if i==1
            try_1 = res;
            y_traj = res.yE;
        else
            try_4 = res;
        end
    end
    % Update
    task.E.vy0 = res.vEy(2);
    task.E.vx0 = res.vEx(2);
    task.E.y0 = res.yE(2);%task.E.y0 + res.vEy(2)*res.ts(1);
    
    % task.E.vref = task.E.vref(2:end); % ref
    % task.N=task.N-1;
    % task.E.horizon=task.E.horizon-1;  %[m] Length of prediction horizon                                 %[m] Length of prediction horizon
    
    % Leading veh update
    task.L.x0 = task.L.x0 + res.ts(1)*task.L.vx - (res.vEx(2)*res.ts(1)) ;
    % Adjacent veh update
    if task.adjacentveh
    task.A.x0 = task.A.x0 + res.ts(1)*task.A.vx - (res.vEx(2)*res.ts(1));
    end
    % 2:nd Adjacent veh update
    if task.adjacentveh2
    task.A2.x0 = task.A2.x0 + res.ts(1)*task.A2.vx - (res.vEx(2)*res.ts(1));
    end
end

%% Backup
% if i == 1 || i==20
% 
%     plotting(task, res, Ax, Ay)
%     if i==1
%         try_1 = res;
%         y_traj = res.yE;
%     else
%         try_4 = res;
%     end
% end
% % Test (remove)
% Ego_pos(end+1) = task.E.y0;
% % Update
% task.E.vref=task.E.vref(2:end); % ref
% task.E.vy0 = res.vEy(2);
% task.E.vx0 = res.vEx(2);
% % Troubleshooting y-axis for host
% 
% task.E.y0 = y_traj(i);
% task.N=task.N-1; 
% task.E.horizon=task.E.horizon-1;  %[m] Length of prediction horizon                                 %[m] Length of prediction horizon
% 
% % Leading veh update
% task.L.x0 = task.L.x0 - res.xE(2);
% %task.L.y0 = task.L.y0 - res.yE(2); %If host-vehicle fixed in origo
% % Adjacent veh update
% task.A.x0 = task.A.x0 - res.xE(2);
% %task.A.y0 = task.A.y0 - res.yE(2); %If host-vehicle fixed in origo