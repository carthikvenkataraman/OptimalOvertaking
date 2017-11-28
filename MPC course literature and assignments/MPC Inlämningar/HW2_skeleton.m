%==========================================================================
% SSY280 Model Predictive Control 2012
%
% Homework Assignment 2:
% MPC Control of a Linearized MIMO Well Stirred Chemical Reactor 
% Revised 2013-02-10
%==========================================================================

%******************* Initialization block *********************************

clear;
close all
tf=50;                  % number of simulation steps

%==========================================================================
% Process model
%==========================================================================

% h = 1; % sampling time in minutes
Ts = 60; % sampling time in seconds 

% Definition of chemical reactor plant 
A = [ 0.2681   -0.00338   -0.00728;
      9.7032    0.3279   -25.44;
         0         0       1   ]; 
B = [ -0.00537  0.1655;
       1.297   97.91 ;
       0       -6.637];
C = [ 1 0 0;
      0 1 0;
      0 0 1];
Bp = [-0.1175;
      69.74;
       6.637 ]; % Disturbances matrix
   
n = size(A,1); % n is the dimension of the state
m = size(B,2); % m is the dimension of the control signal
p = size(C,1); % p is the dimension of the measured output

d=0.01*[zeros(1*tf/5,1);ones(4*tf/5,1)]; % unmeasured disturbance trajectory

x0 = [0.01;1;0.1]; % initial condition of system's state


%==========================================================================
% Set up estimated model
%==========================================================================

% Three cases to be investigated

example = 'c';
switch example
    case 'a'
        nd = 2;
        Bd = zeros(n,nd);
        Cd = [1 0;0 0; 0 1]; 
        q=1; r=1; 
    case 'b'
        nd=3;
        Bd = zeros(n,nd); 
        Cd = [1 0 0;0 0 1;0 1 0];
        q=1000; r=1; 
    case 'c'
        nd=3; 
        Bd = [zeros(3,2) Bp];
        Cd = [1 0 0;0 0 0;0 1 0];
        q=1; r=1; 
end


    
% Augment the model with constant disturbances

Ae = [A Bd; zeros(nd,n) eye(nd)]; % 
Be = [B ; zeros(nd,m)]; %  
% Be_2 = [zeros(n,nd);eye(nd)];
G = eye(n+nd);
Be_new = [Be, G]; 
Ce = [C Cd];

% Check augumented system stability 
if rank([eye(n)-A -Bd;C Cd]) == n+nd 
    disp('augumented system is stable')
else
    disp('augumented system is not stable')
end
% Calculate estimated gain 

sysd = ss(Ae,Be_new,Ce,[],Ts);
Qk = eye(n+nd)*q;   % Varience of process noise  
Rk = eye(n)*r;   % Variance of measurement noise
NN = 0;          % Covariance of measurement noise and process noise
[kest,Le,P] = kalman(sysd,Qk,Rk,NN,'delayed');  

% Check estimator stability poles <= 1
estimator_poles = eig(Ae-Le*Ce)
abs(estimator_poles)

%==========================================================================
% Prepare for computing steady state targets
%==========================================================================

% Select 1st and 3rd outputs as controlled outputs   
H = [1 0 0;0 0 1]; 

% Matrices for steady state target calculation to be used later

Ta = [eye(n)-A , -B;
    H*C zeros(size(H,1),m)];


% YOUR CODE GOES HERE

%==========================================================================
% Set up MPC controller
%==========================================================================

N=10;                   % prediction horizon
M=3;                    % control horizon

Q = diag([1 0.001 1]);  % state penalty
Pf = Q;                 % terminal state penalty
R = 0.01*eye(m);        % control penalty
    
    %=================================
    % Build Hessian Matrix
    %=================================

    %YOUR MODIFIED CODE FROM ASSIGNMENT 1 GOES HERE
Q_bar=kron(eye(N-1),Q); 
R_bar=kron(eye(M),R); 
% Hessian matrix
HM=(blkdiag(Q_bar,Pf,R_bar)); 
    %==========================================
    % Equality Constraints
    %==========================================
    
Nn = kron(eye(N-1), A);
nn = zeros(n,n*N-n);
nN = [nn;Nn];
NN = [nN,zeros(n*N,n)];
Mn = kron(eye(M),B);
MM = zeros(n*(N-M),m*(M-1));
BM = repmat(B,N-M,1); 
mM = [Mn;MM,BM];

Aeq = [kron(eye(N),eye(n)) + NN,mM];
AA = [-A;zeros(n*N-n,n)];

    %  YOUR MODIFIED CODE FROM ASSIGNMENT 1 GOES HERE
    
    %==========================================
    % Inequality Constraints
    %==========================================
    
    Ain = [];
    Bin = [];
    
    %==============================================
    % Choose QP solver 
    %==============================================
    
    solver = 'int';
    switch solver
        case 'int'
            options = optimset('Algorithm','interior-point-convex','Display','off');
        case 'set'
            options = optimset('Algorithm','active-set','Display','off');
    end

%******************* End of initialization ********************************    

%==========================================================================
% Simulation
%==========================================================================
    
% Initialization
    
% YOUR CODE GOES HERE - INITIALIZE ALL VARIABLES NEEDED 
dhat = zeros(nd,1); % initialize estimated disturbaces  
xdhat = [x0;dhat];  % initialize estimated state vector and disturbaces
xk = x0;            % initial state value for process
yk = C*xk;          % initial output value for process
zsp = zeros(m,1);   % Setpoint 
uk = zsp;           % initial input value for process

% Construct matrixes to save values in 
xdhat_save = zeros(n+nd,tf);    
z_target_save = zeros(n+m,tf);
uk_save = zeros(m,tf);
yk_save = zeros(n,tf);
xk_save = zeros(n,tf);
delta_x_save = zeros(n,tf); 
delta_u_save = zeros(m,tf);

% Simulate closed-loop system 
    
    for k = 1:tf

        %======================================
        % Update the estimated state xhat(k|k-1)
        %======================================
        
        xdhat = Ae*xdhat + Be*uk + Le*(yk-Ce*xdhat);
        
        %==============================================
        % Update the process state x(k) and output y(k)
        %==============================================
        
        xk = A*xk + B*uk + Bp*d(k);
        yk = C*xk;        
        
        %=========================================
        % Calculate steady state targets xs and us
        %=========================================
        
        dhat = xdhat(n+1:end);
        
        Tb = [Bd*dhat;
            zsp-H*Cd*dhat];
        
        z_target = Ta\Tb;
        
        %============================================
        % Solve the QP (for the deviation variables!)
        %============================================
        xhat = xdhat(1:n);
        delta_x = xhat-z_target(1:n);
        beq =AA*delta_x;
        % UPDATE RHS OF EQUALITY CONSTRAINT HERE
        
        % NOTE THAT HM IS USED FOR THE HESSIAN, NOT TO BE CONFUSED 
        %   WITH H THAT IS USED FOR SELECTING CONTROLLED VARIABLES
        z = quadprog(HM,[],Ain,Bin,Aeq,beq,[],[],[],options);
       
        % CALCULATE THE NEW CONTROL SIGNAL HERE
         delta_u = z(n*N+1:n*N+m);
        uk = delta_u+z_target(n+1:n+m);
        % NOTE THAT YOU NEED TO GO FROM DEVIATION VARIABLES TO 'REAL' ONES!
        
        %===============================        
        % Store current variables in log 
        %===============================
        xdhat_save(:,k) = xdhat;
        z_target_save(:,k) = z_target;
        uk_save(:,k) = uk;
        yk_save(:,k) = yk;
        xk_save(:,k) = xk;
        delta_x_save(:,k) = delta_x; 
        delta_u_save(:,k) = delta_u;
        
   end % simulation loop
%==========================================================================
% Plot results
%==========================================================================
textsize = 13;
t = (1:1:tf);

figure;
subplot(3,1,1); 
plot(t,xdhat_save(1,:),'r',t,xk_save(1,:),'b'); % concentration subst A
grid on;
set(gca,'fontsize',textsize)
legend({'$\hat{c}$','c'},'Interpreter','latex')
title({'Estimated concentration of substaces A, $\hat{c}$ and real concentration of A, c'},'Interpreter','latex')
ylabel({'Concentration','of substaces A'},'Interpreter','latex')% not given anny units o this 
xlabel('Time [min]','Interpreter','latex')

subplot(3,1,2); 
plot(t,xdhat_save(2,:),'r',t,xk_save(2,:),'b'); % Temperature
grid on;
set(gca,'fontsize',textsize)
legend({'$\hat{T}$','T'},'Interpreter','latex')
title({'Estimated Temperature, $\hat{T}$ and real Temperature, T'},'Interpreter','latex')
ylabel({'Temperature'},'Interpreter','latex')% not given anny units o this 
xlabel('Time [min]','Interpreter','latex')

subplot(3,1,3);
plot(t,xdhat_save(3,:),'r',t,xk_save(3,:),'b'); % Tank level
grid on;
set(gca,'fontsize',textsize)
legend({'$\hat{h}$','h'},'Interpreter','latex')
title({'Estimated tank level, $\hat{h}$ and real tank level, h'},'Interpreter','latex')
ylabel({'Tank level'},'Interpreter','latex')% not given anny units o this 
xlabel('Time [min]','Interpreter','latex')


figure;
subplot(3,1,1); 
plot(t,delta_x_save(1,:),'r'); % concentration subst A
grid on;
set(gca,'fontsize',textsize)
legend({'$\delta h$'},'Interpreter','latex');
title({'Difference between estimated and steady state','concentration of substances A, $\delta c = \hat{c}-c^s$'},'Interpreter','latex')
ylabel({'$\delta c = \hat{c}-c^s$'},'Interpreter','latex')% not given anny units o this 
xlabel('Time [min]','Interpreter','latex')


subplot(3,1,2); 
plot(t,delta_x_save(2,:),'r'); % Temperature
grid on;
set(gca,'fontsize',textsize)
legend({'$\delta T$'},'Interpreter','latex');
title({'Difference between estimated and steady state','of Temparature $\delta T = \hat{T}-T^s$'},'Interpreter','latex')
ylabel({'$\delta T = \hat{T}-T^s$'},'Interpreter','latex')% not given anny units o this 
xlabel('Time [min]','Interpreter','latex')

subplot(3,1,3);
plot(t,delta_x_save(3,:),'r'); % Tank level
grid on;
set(gca,'fontsize',textsize)
legend({'$\delta h$'},'Interpreter','latex');
title({'Difference between estimated and steady state','of tank level $\delta h = \hat{h}-h^s$'},'Interpreter','latex')
ylabel({'$\delta h = \hat{h}-h^s$'},'Interpreter','latex')% not given anny units o this 
xlabel('Time [min]','Interpreter','latex')

figure;
subplot(3,1,1); 
plot(t,xdhat_save(1,:),'r',t,z_target_save(1,:),'b'); % concentration subst A
grid on;
set(gca,'fontsize',textsize)
legend({'$\hat{c}$','$c^s$'},'Interpreter','latex')
title({'Estimated $\hat{c}$ and steady state target $c^s$ concentration of A'},'Interpreter','latex')
ylabel({'Concentration','of substaces A'},'Interpreter','latex')% not given anny units o this 
xlabel('Time [min]','Interpreter','latex')

subplot(3,1,2); 
plot(t,xdhat_save(2,:),'r',t,z_target_save(2,:),'b'); % Temperature
grid on;
set(gca,'fontsize',textsize)
legend({'$\hat{T}$','$T^s$'},'Interpreter','latex')
title({'Estimated $\hat{T}$ and steady state target $T^s$ of Temperature'},'Interpreter','latex')
ylabel({'Temperature'},'Interpreter','latex')% not given anny units o this 
xlabel('Time [min]','Interpreter','latex')

subplot(3,1,3);
plot(t,xdhat_save(3,:),'r',t,z_target_save(3,:),'b'); % Tank level
grid on;
set(gca,'fontsize',textsize)
legend({'$\hat{h}$','$h^s$'},'Interpreter','latex')
title({'Estimated $\hat{h}$ and steady state target $h^s$ of tank level'},'Interpreter','latex')
ylabel({'Tank level'},'Interpreter','latex')% not given anny units o this 
xlabel('Time [min]','Interpreter','latex')


% fig_num =1; 
% figure(fig_num)
% fig_num = 1 + fig_num;
% subplot(3,1,1) % For the other two sets of parameters you should change
% % the third index to 2 and 3, respectively.
% plot(tvec,z_target_save(1:n),'-',tvec,z_target_save(n+1:m),'--'); grid
% %axis([0 10 -4 2])
% set(gca,'fontsize',textsize)
% legend('','u(k) = \tau(k) [Nm]');
% title(['Without constraints & parameaters N = ',num2str(N),', q = ',...
%     num2str(q),' & r = ',num2str(r)]);
% xlabel('Time [s]');
% ylabel('Torque [Nm], Angle [rad]');


