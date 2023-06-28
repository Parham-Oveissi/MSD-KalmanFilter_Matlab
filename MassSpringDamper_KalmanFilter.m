clc; clear; close all
m = 10; % mass > 0
k = 5; % spring constant > 0
b = 3; % damping coefficient ≥ 1
A = [0 1; -k/m -b/m]; 
B = [0; 1/m];
Force = 1;


Process_STD = 0.01;
Q = Process_STD^2;
B_Aug = [B randn(2,1)];
C = [1 0];
D = 0;

dt = 0.2;
t = 0:dt:50;

u = Force; 
u_AUG = [u; Q];

% mysys = ss(A,B_Aug,C,D);
% [y,t] = lsim(mysys,u_AUG,t);
xinit = [1;0];
Dynamics_with_disturbance = @(t,x) A*x+B_Aug*u_AUG; % rhs of function
[~,Disturbed_States] = ode45(Dynamics_with_disturbance, t, xinit);

Dynamics_without_disturbance = @(t,x) A*x+B*u; % rhs of function
[~,True_States] = ode45(Dynamics_without_disturbance, t, xinit);


mysys_continous = ss(A,B,C,D);
mysys_discrete = c2d(mysys_continous,dt,'zoh');

Measurement_noise_std = 0.1; % Measurement noise std
Measurement = Disturbed_States(:,1); 
Measurement_noisy = Measurement + Measurement_noise_std*randn(size(Measurement)); % Add noise to the Measurements


Estimate = [3;3]; % Initial state Estimate
P = 400*eye(length(Estimate)); % Initial state Covariance
Estimate_Var = diag(P); % Initial state variance


F = mysys_discrete.A; % State transition matrix
G = mysys_discrete.B; % Input/Control transition Matrix
H = mysys_discrete.C; % Measurement Matrix

Q_Matrix = Q*eye(2); %Innovation or system error
R = Measurement_noise_std^2; %Measurement error
Estimates_Vec(:,1) = Estimate;
for i = 2:length(Measurement)
    % Prediction step
    Estimate = F*Estimate + G*u;
    P = F*P*F' + Q_Matrix;
    % Observation update
    K = (P*H')/(H*P*H'+R);
    Estimate = Estimate + K*(Measurement_noisy(i) - H*Estimate);
    P = P - K*H*P;
    Estimates_Vec(:,i) = Estimate;
end
 

figure;subplot(2,1,1);
plot(t,True_States(:,1),'g','linewidth',3);
hold on;
plot(t,Measurement_noisy,'bo','markerfacecolor','b','markersize',6)
plot(t,Estimates_Vec(1,:),'k','linewidth',3);
ylabel('Position','fontsize',14)
legend('True States','Noisy Measurements','Estimates')
subplot(2,1,2);
plot(t,True_States(:,2),'g','linewidth',3);
hold on;
plot(t,Estimates_Vec(2,:),'k','linewidth',3);
xlabel('Time','fontsize',14)
ylabel('Velocity','fontsize',14)
legend('True States','Estimates')