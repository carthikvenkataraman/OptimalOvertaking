function res = solveOvertake(task)

%   x is state vector:  
%   x(1) = z_Ex;            [s/m] inverse longitudinal velocity of the ego vehicle
%   x(2) = y_E;             [m/s] longitudinal velocity of the ego vehicle
%   x(3) = t_E;             [m]   lateral position of the ego vehicle

%   u is control vector:
%   u(1) = u_1=F_E*z_E^3/m; [s/m^2] input 1 FEL?? u och dy ist
%   u(2) = v_Ey;            [m/s]   lateral velocity of the ego vehicle

ds      = task.ds;
N       = task.N;
yr      = task.E.yref;
road    = task.road;
vL      = task.L.vx;
vr      = task.E.vref-vL;           % Speed of ego vehicle relative to pilot
vrmean  = mean(vr);
slip    = tan(task.E.maxslipangle);
m       = task.E.weight;
epsilon = task.E.vxmin+0.01;
Fr      = task.E.Fr;

%% Scaling factors - transformation from time to space domain

Sy   = max(task.zone.ymax);
Sdy  = task.E.vymax/vrmean/2;
Sddy = task.E.aymax/vrmean^2; 
Sz   = 1/(task.E.vxmin+epsilon);
Su   = task.E.axmax/vrmean^3;
Sdu  = 2e-2;
Scost= 10*vrmean; 
St   = task.s(end)/vrmean*1.5;

%% Penalties

Wdx  =0.01*vrmean;
WF=2/vrmean;
Wdddx=100/vrmean^3;                                   % Initial time domain weights

Wz=Wdx*vrmean^3;
Wy=0.1;
Wu=WF*vrmean^5;
Wdy=50;
Wdu=Wdddx*vrmean^7;
Wddy=800;                                             % Transformed to correspond to inverse domain

%% Linearization

zlin = 1/vrmean*ones(N,1);                            % State to linear around

%% Yalmip

z   = sdpvar(N,1);
y   = sdpvar(N,1);
u   = sdpvar(N-1,1);
dy  = sdpvar(N-1,1);
ddy = (dy(2:N-1)-dy(1:N-2))/ds*Sdy/Sddy;              % d^2y/ds^2
du  = (u(2:N-1)-u(1:N-2))/(ds)*Su/Sdu;                % d^2v/ds^2

solvertime = 0;

if task.adjacentveh || task.oncomingveh
    t = sdpvar(N,1);
else 
    t = zeros(N,1);
end

options = sdpsettings('solver','QuadProg','verbose',0);

%% Objective / cost function
        
cost = (sum((y*Sy-yr).^2)*Wy + sum(dy.^2)*Wdy*Sdy^2 ...
    + sum(ddy.^2)*Wddy*Sddy^2 + sum(du.^2)*Wdu*Sdu^2 ...
    + sum((z*Sz - 1./vr).^2)*Wz + sum(u.^2)*Wu*Su^2)*ds/Scost;
    
%% Equality Constraints - system dynamics - C1

if task.adjacentveh || task.oncomingveh
    C1=[...
        z(2:N)==z(1:N-1)-u(1:N-1)*ds*Su/Sz...
        y(2:N)==y(1:N-1)+ds*dy(1:N-1)*Sdy/Sy...
        t(2:N)==t(1:N-1)+ds*z(1:N-1)*Sz/St...
        ];
else
    C1=[...
        z(2:N)==z(1:N-1)-u(1:N-1)*ds*Su/Sz...
        y(2:N)==y(1:N-1)+ds*dy(1:N-1)*Sdy/Sy...
        ];
end    
    
%% Inequality constraints - C2

C2=[...
    y >= task.zone.ymin/Sy ...
    y <= task.zone.ymax/Sy ...
    z >= 1/(task.E.vxmax-vL)/Sz...
    z <= 1/epsilon/Sz...
    u >= (task.E.axmin-Fr/m)*(zlin(2:N).^3+3*zlin(2:N).^2.*(z(2:N)*Sz-zlin(2:N)))./Su ...
    u <= (task.E.axmax-Fr/m)*(zlin(2:N).^3+3*zlin(2:N).^2.*(z(2:N)*Sz-zlin(2:N)))./Su ...
    dy >= -slip*(1+vL*z(2:N)*Sz)/Sdy...
    dy <=  slip*(1+vL*z(2:N)*Sz)/Sdy...
    z(1) == 1/(task.E.vx0 - vL)/Sz, ...
    y(1) == task.E.y0/Sy ...
    y(N) == task.E.yref(end)/Sy ... 
%     dy(1) == task.E.vy0/Sdy ... We do not want zero vel in each iteration
    u(1) ==  0/Su ...
    t(1) == 0/St ...
    ];

%% Other Inequality constraints - C3

% if task.oncomingveh && task.egofirst % Not of interest, ONCOMING
%     C3=y(task.zone.ixov) <= (task.O.y0 - road.lanewidth - road.lanewidth/task.O.lf*(task.s(task.zone.ixov)-task.O.x0 + (vL-task.O.vx)*t(task.zone.ixov)*St))/Sy;
% elseif task.oncomingveh && ~task.egofirst % Not of interest, ONCOMING
%     C3=y(task.zone.ixov) <= (task.O.y0 - road.lanewidth + road.lanewidth/task.O.lf*(task.s(task.zone.ixov)-task.O.x0 + (vL-task.O.vx)*t(task.zone.ixov)*St))/Sy;

if task.adjacentveh && task.ego==1                                          % infront of first vehicle
    C3 = y(task.zone.ixov) <= ...
        (task.A.y0 - road.lanewidth - road.lanewidth/task.A.lr*(task.s(task.zone.ixov)-...
        task.A.x0 + (vL-task.A.vx)*t(task.zone.ixov)*St))/Sy;
elseif task.adjacentveh && task.ego==2                                      % Behind first veh and infront of second vehicle
    C3 = [y(task.zone.ixov) <= ...
        (task.A.y0 + road.lanewidth - road.lanewidth/task.A.lr*(task.s(task.zone.ixov)-...
        task.A.x0 + (vL-task.A.vx)*t(task.zone.ixov)*St))/Sy...
          y(task.zone.ixov) <= ...
          (task.A2.y0 - road.lanewidth - road.lanewidth/task.A2.lr*(task.s(task.zone.ixov)-...
          task.A2.x0 + (vL-task.A2.vx)*t(task.zone.ixov)*St))/Sy];    
elseif task.adjacentveh && task.ego==3                                      % Behind 2:nd vehicle
    C3 = y(task.zone.ixov) <=...
        (task.A2.y0 + road.lanewidth - road.lanewidth/task.A2.lr*(task.s(task.zone.ixov)-...
         task.A2.x0 + (vL-task.A2.vx)*t(task.zone.ixov)*St))/Sy;    
else
    C3 = [];
end
    
C = [C1 C2 C3];

%% Optimisation and population of output struct

resOpt          = optimize(C,cost,options);

res             = struct;
res.status      = resOpt;
res.solvertime  = solvertime;
res.cost.total  = value(cost);
res.yE          = value(y*Sy);                                              % [m] lateral position
res.vEx         = value(1./(z*Sz)+vL);                                      % [m/s] longitudinal speed
res.vEy         = [value(dy); value(dy(N-1))]*Sdy.*(res.vEx-vL);            % [m/s] lateral speed
res.ts          = value(ds./(res.vEx-vL));                                  % [s] time samples
res.t           = [0;cumsum(res.ts(1:N-1))];                                % [s] travel time
res.xE          = [task.E.x0; cumsum(res.vEx(1:N-1).*diff(res.t))];         % [m] absolute longitudinal position
% res.xE          = task.s + vL*res.t;                                      % [m] longitudinal position
res.topt        = value(t)*St;
res.weights     = [Wz Wy Wu Wdy Wdu Wddy];
res.z           = value(z*Sz);
res.Ax          = diff(res.vEx)./diff(res.t);
res.Ax          = [res.Ax; res.Ax(end)];
res.Ay          = diff(res.vEy)./diff(res.t);
res.Ay          = [res.Ay; res.Ay(end)];
res.Jx          = diff(res.Ax)./diff(res.t);