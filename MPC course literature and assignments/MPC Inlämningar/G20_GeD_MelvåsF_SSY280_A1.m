%%---Basic system model
%% Without constraints
clear all
clc
close all
fig_num =1; 
textsize = 10;
%
h=0.1;
A=[1 h;0.5*h 1];
B=[h^2/2; h];
C=[1 0];
n=size(A,1);
m=size(B,2);
%
%---Parameters
%
x0=[0.5 1]';
q_vec = [3.8 3.8 10];
r_vec = [1 1 1];
N_vec = [5 10 10];
%
for l = 1:length(q_vec)
q = q_vec(l);
r = r_vec(l);
N = N_vec(l);
%---Define matrices for the QP
Q=q*(C'*C);
R=r;
P=q*eye(n);
Q_bar=kron(eye(N-1),Q);
R_bar=kron(eye(N),R);
%
H=2*(blkdiag(Q_bar,P,R_bar));
f=[];
%---For the case with actuator constraints
%w2
Ain=[]; % Use empty matrices for the first case without actuator...
bin=[]; % ...constraints and change for the case with constraints!
%
%---Cost function
Nn = kron(eye(N-1), -A);
nn = zeros(2,2*N-2);
nN = [nn;Nn];
NN = [nN,zeros(2*N,2)];
Aeq = [kron(eye(N),eye(2)) + NN,kron(eye(N),-B)];
%
%---MPC algorithm
%
T=100; % simulation time
%
xk=x0; % initialize state vector
yvec=[];
uvec=[];
options = optimset('Algorithm','interior-point-convex','Display','off');
%
AA = [A;zeros(2*N-2,2)];
%
tic;
for k=1:T
beq=AA*xk; % The matrix AA defines how the last measured state xk
% determines the right hand side of the inequality condition.
tic;
z=quadprog(H,f,Ain,bin,Aeq,beq,[],[],[],options);
t_uncon(k,l) = toc;
uk=z(n*N+1);
xk=A*xk+B*uk;
yvec=[yvec; C*xk];
uvec=[uvec; uk];
end
t_loop_uncon(l) = toc;
tvec=h*(1:1:T);
fig_num =1; 
figure(fig_num)
fig_num = 1 + fig_num;
subplot(3,1,l) % For the other two sets of parameters you should change
% the third index to 2 and 3, respectively.
plot(tvec,yvec,'-',tvec,uvec,'--'); grid
axis([0 10 -4 2])
set(gca,'fontsize',textsize)
legend('y(k) = \theta(k) [rad]','u(k) = \tau(k) [Nm]');
title(['Without constraints & parameaters N = ',num2str(N),', q = ',...
    num2str(q),' & r = ',num2str(r)]);
xlabel('Time [s]');
ylabel('Torque [Nm], Angle [rad]');

if l == 2
figure(fig_num)  
subplot(3,1,1) % For the other two sets of parameters you should change
% the third index to 2 and 3, respectively.
plot(tvec,yvec,'-',tvec,uvec,'--'); grid
axis([0 10 -4 1])
set(gca,'fontsize',textsize)
legend('y(k) = \theta(k) [rad]','u(k) = \tau(k) [Nm]');
title(['Without constraints & parameaters N = ',num2str(N),', q = ',...
    num2str(q),' & r = ',num2str(r)]);
xlabel('Time [s]');
ylabel('Torque [Nm], Angle [rad]');
end
    
end

% Part C
%---Parameters
%
%
q = 3.8;
r = 1;
N = 10;
%---Define matrices for the QP
Q=q*(C'*C);
R=r;
P=q*eye(n);
Q_bar=kron(eye(N-1),Q);
R_bar=kron(eye(N),R);
%
H=2*(blkdiag(Q_bar,P,R_bar));
f=[];
%---For the case with actuator constraints
%w2
Ain=[]; % Use empty matrices for the first case without actuator...
bin=[]; % ...constraints and change for the case with constraints!
%
%---Cost function
Nn = kron(eye(N-1), -A);
nn = zeros(2,2*N-2);
nN = [nn;Nn];
NN = [nN,zeros(2*N,2)];
Aeq = [kron(eye(N),eye(2)) + NN,kron(eye(N),-B)];
%
%---Ricatti algorithm
%
T=100; % simulation time
p=P;
for i=fliplr(1:N-1)
    p=Q+A'*p*A-(A'*p*B)*((R+B'*p*B)\(B'*p*A)); 
end
K = -(R+B'*p*B)\(B'*p*A);
xk=x0; % initialize state vector
 yvec=[];
 uvec=[];
for k=1:T
uk=K * xk;
xk=A*xk+B*uk;
yvec=[yvec; C*xk];
uvec=[uvec; uk];
end

figure(fig_num)

subplot(3,1,2) % For the other two sets of parameters you should change
% the third index to 2 and 3, respectively.
plot(tvec,yvec,'-',tvec,uvec,'--'); grid
axis([0 10 -4 1])
set(gca,'fontsize',textsize)
legend('y(k) = \theta(k) [rad]','u(k) = \tau(k) [Nm]');
title(['Ricatti recursions N = ',num2str(N),', q = ',num2str(q),...
    ' & r = ',num2str(r)]);
xlabel('Time [s]');
ylabel('Torque [Nm], Angle [rad]');
%
q = 3.8;
r = 1;
N = 100;
%---Define matrices for the QP
Q=q*(C'*C);
R=r;
P=q*eye(n);
Q_bar=kron(eye(N-1),Q);
R_bar=kron(eye(N),R);
%
H=2*(blkdiag(Q_bar,P,R_bar));
f=[];
%---For the case with actuator constraints
%w2
Ain=[]; % Use empty matrices for the first case without actuator...
bin=[]; % ...constraints and change for the case with constraints!
%
%---Cost function
Nn = kron(eye(N-1), -A);
nn = zeros(2,2*N-2);
nN = [nn;Nn];
NN = [nN,zeros(2*N,2)];
Aeq = [kron(eye(N),eye(2)) + NN,kron(eye(N),-B)];
%
%---MPC algorithm
%
T=100; % simulation time
%
xk=x0; % initialize state vector
yvec=[];
uvec=[];
options = optimset('Algorithm','interior-point-convex','Display','off');
%
AA = [A;zeros(2*N-2,2)];
beq=AA*x0; % The matrix AA defines how the last measured state xk
% determines the right hand side of the inequality condition.
%
z=quadprog(H,f,Ain,bin,Aeq,beq,[],[],[],options);
for k=1:T
uk=z(n*N+k);
xk=A*xk+B*uk;
yvec=[yvec; C*xk];
uvec=[uvec; uk];
end
%
subplot(3,1,3) % For the other two sets of parameters you should change
% the third index to 2 and 3, respectively.
plot(tvec,yvec,'-',tvec,uvec,'--'); grid
axis([0 10 -4 1])
set(gca,'fontsize',textsize)
legend('y(k) = \theta(k) [rad]','u(k) = \tau(k) [Nm]');
title(['Optimiation of u(k) before loop N = ',num2str(N),', q = ',num2str(q),...
    ' & r = ',num2str(r)]);
xlabel('Time [s]');
ylabel('Torque [Nm], Angle [rad]');
%
fig_num = 1 + fig_num;
%
%% With constraints
exist fig_num
if ans == 1
else
clear all
clc
close all
fig_num =1; %use if run only this section 
textsize = 12;
end
%
h=0.1;
A=[1 h;0.5*h 1];
B=[h^2/2; h];
C=[1 0];
n=size(A,1);
m=size(B,2);
%---Parameters
x0=[0.5 1]';
q_vec = [3.8 3.8 10];
r_vec = [1 1 1];
N_vec = [5 10 10];
%
for l = 1:length(q_vec)
q = q_vec(l);
r = r_vec(l);
N = N_vec(l);
%---Define matrices for the QP
Q=q*(C'*C);
R=r;
P=q*eye(n);
Q_bar=kron(eye(N-1),Q);
R_bar=kron(eye(N),R);
%
H=2*blkdiag(Q_bar,P,R_bar);
f=[];
%---For the case with actuator constraints
%
Ain=[zeros(N,2*N),eye(N);zeros(N,2*N),-eye(N)]; % Use empty matrices for...
bin=[ones(1,N)';ones(1,N)']; % ... the first case without actuator 
% constraints ... and change for the case with constraints!
%---Cost function
Nn = kron(eye(N-1), -A);
nn = zeros(2,2*N-2);
nN = [nn;Nn];
NN = [nN,zeros(2*N,2)];
Aeq = [kron(eye(N),eye(2)) + NN,kron(eye(N),-B)];
%
%---MPC algorithm
%
 T=100; % simulation time
% 
 xk=x0; % initialize state vector
 yvec=[];
 uvec=[];
 options = optimset('Algorithm','interior-point-convex','Display','off');
%options = optimset('Algorithm','active set','Display','off');
%
 AA = [A;zeros(2*N-2,2)];
tic;
for k=1:T
beq=AA*xk; % The matrix AA defines how the last measured state xk
% determines the right hand side of the inequality condition.
tic;
z=quadprog(H,f,Ain,bin,Aeq,beq,[],[],[],options);
t_con(k,l) = toc;
uk=z(n*N+1);
xk=A*xk+B*uk;
yvec=[yvec; C*xk];
uvec=[uvec; uk];
end
t_loop_con(l) = toc; 
 tvec=h*(1:1:T);
figure(fig_num)
%
subplot(3,1,l) % For the other two sets of parameters you should change
% the third index to 2 and 3, respectively.
plot(tvec,yvec,'-',tvec,uvec,'--'); grid
axis([0 10 -1.5 7])
legend('y(k) = \theta(k) [rad]','u(k) = \tau(k) [Nm]');
title(['With constraints & parameaters N = ',num2str(N),', q = ', ...
    num2str(q),' & r = ',num2str(r)]);
set(gca,'fontsize',textsize)
xlabel('Time [s]');
ylabel('Torque [Nm], Angle [rad]');
end
fig_num = 1 + fig_num;
%
%% E To run first run With and withoute con
exist fig_num
if ans == 1
else
error('Run secton with and withoute constrains first')
end
figure(fig_num)
fig_num = 1 + fig_num;
plot(tvec,t_uncon(:,3),tvec,t_con(:,3))
set(gca,'fontsize',textsize)
axis([0 10 0 1.2*max(t_con(:,3))])
legend('Un constrained case','Constrained case')
title(['Execution times for minimization N = ',num2str(N) ...
    ,', q = ',num2str(q),' & r = ',num2str(r)'])
ylabel('Execution time [s]')
xlabel('Simulation time [s]')