function res=solveFollow(task)

%   x is state vector:  
%   x(1) = x;               [s/m] inverse longitudinal velocity of the ego vehicle
%   x(2) = vx;              [m/s] longitudinal velocity of the ego vehicle
%   x(3) = y;               [m]   lateral position of the ego vehicle

%   u is control vector:
%   u(1) = u_1=F_E*z_E^3/m; [s/m^2] input 1 FEL?? u och dy ist
%   u(2) = v_Ey;            [m/s]   lateral velocity of the ego vehicle

dt = task.dt;
N = task.N;
yr = task.E.yref;
vL = task.L.vx;
vr = task.E.vref;

%% Yalmip

% States
x = sdpvar(N,1);
y = sdpvar(N,1);
vx = sdpvar(N,1);

% Inputs
ax=sdpvar(N-1,1);
vy=sdpvar(N-1,1);

% Variables for cost fcn
dax=(ax(2:end)-ax(1:end-1))/dt;                                 % derivative on acc in x
ay=(vy(2:end)-vy(1:end-1))/dt;                                  % acc in y

k=1;
solvertime=0;

options = sdpsettings('solver','QuadProg','verbose',3);

%% Objective / cost function

% Weights
q1 = 1;
q2 = 1;
q3 = 1;
q4 = 1;
q5 = 1;
q6 = 1;

cost = (sum((vx-(vr-task.L.vx)).^2)*q1 + (sum((y-yr).^2))*q2 ...
    + sum(ax.^2)*q3 + sum(vy.^2)*q4 ...
    + sum(dax.^2)*q5 + sum(ay.^2)*q6);

%% Equality Constraints - system dynamics - C1

C1=[...
    x(2:N)==x(1:N-1) + vx(1:N-1)*dt...
    vx(2:N)==vx(1:N-1) + ax(1:N-1)*dt...
    y(2:N)==y(1:N-1)+vy(1:N-1)*dt...
    ];

%% Inequality constraints - C2

C2=[...
    x <= -task.L.longsafetymargin ...
    vx <= -x/dt ...
    y <= task.zone.ymax ...
    vy >= task.E.vymin ...                                  % maybe include speed dependent
    vy <= 0 ...                                             % To only allow right turns, otherwise: task.E.vymax ...
    x(1) == task.E.x0 - task.L.x0...
    vx(1) == task.E.vx0 - task.L.vx...
    vx(N) == 0 ...
    y(1) == task.E.y0 ...
    y(N) == task.E.yref(end) ...
    vy(1) == task.E.vy0 ...
    ];
    
C=[C1 C2];

%% Optimisation and population of output struct


resOpt = optimize(C,cost,options);

res = struct;
res.status = resOpt;
res.solvertime = solvertime;
res.cost.total = value(cost);
res.yE = value(y);                                          %[m] lateral position
res.vEx = value(vx);                                        %[m/s] longitudinal speed
res.vEy = [value(vy); value(vy(N-1))];                      %[m/s] lateral speed
res.ts = ones(N,1)*dt;                                      %[s] time samples
res.t = [0;cumsum(res.ts(1:N-1))];                          %[s] travel time
res.xE = value(x);                                          %[m] longitudinal position
%res.topt=value(t)*St;
res.weights = [q1 q2 q3 q4 q5 q6];
%res.z=value(z*Sz);