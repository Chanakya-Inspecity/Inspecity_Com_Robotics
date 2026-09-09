%clear;
clc;
close all;

%% Simulation Time
t0 = 0;
tf = 20;
tspan = [t0 tf];

%% Initial Conditions

q0  = [-pi/2; 0; 0];   % start at hanging equilibrium
dq0 = [0; 0; 0];

state0 = [q0;
          dq0];

%% Pure time-dependent torques — NO angle feedback (identical input to both models)
% Designed within 80% of Lyapunov stable region.
% Max gravity restoring: J1=25.5 Nm, J2=8.34 Nm, J3=1.47 Nm
% Binary-search verified (scale=0.375 on [8;3;0.8] direction, dt=2e-3, 20s):
%   dev=[17.1°, 31.1°, 36.4°] — all within 80% limits (72°, 39.2°, 72°)
% 80% limits: q1∈(-162°,-18°), |q2|<39.2°, |q3|<72°

tau_fun = @(t, ~) [ ...
    3.0 * sin(0.30*t); ...
    1.0 * sin(0.50*t + pi/4); ...
    0.3 * sin(0.70*t + pi/3)];

%% ODE45 Solver

options = odeset('RelTol',1e-8,'AbsTol',1e-10);
dt      = 1e-4;
tspan   = (0:dt:20);

[t,state] = ode45(@(t,s) NE_FD(s, tau_fun(t,s)), tspan, state0, options);

%% Extract States

q  = state(:,1:3);
dq = state(:,4:6);

%% Compute Joint Accelerations

N = length(t);

ddq = zeros(N,3);
tau = zeros(N,3);

for k = 1:N
    tau(k,:) = tau_fun(t(k), state(k,:).').';
    xdot     = NE_FD(state(k,:).', tau(k,:).');
    ddq(k,:) = xdot(4:6).';
end


%% Save variables
%% Save variables in Analytical format

an_tout = t;

an_q1 = q(:,1);
an_q2 = q(:,2);
an_q3 = q(:,3);

an_dq1 = dq(:,1);
an_dq2 = dq(:,2);
an_dq3 = dq(:,3);

an_ddq1 = ddq(:,1);
an_ddq2 = ddq(:,2);
an_ddq3 = ddq(:,3);

an_tau1 = tau(:,1);
an_tau2 = tau(:,2);
an_tau3 = tau(:,3);

save('Analytical_NE_Output.mat', ...
    'an_tout', ...
    'an_q1','an_q2','an_q3', ...
    'an_dq1','an_dq2','an_dq3', ...
    'an_ddq1','an_ddq2','an_ddq3', ...
    'an_tau1','an_tau2','an_tau3');

%% Plot Results

figure;
subplot(3,1,1)
plot(t,q,'LineWidth',1.5)
grid on
xlabel('Time (s)')
ylabel('q (rad)')
legend('q1','q2','q3','Location','best')

subplot(3,1,2)
plot(t,dq,'LineWidth',1.5)
grid on
xlabel('Time (s)')
ylabel('dq (rad/s)')
legend('dq1','dq2','dq3','Location','best')

subplot(3,1,3)
plot(t,ddq,'LineWidth',1.5)
grid on
xlabel('Time (s)')
ylabel('ddq (rad/s^2)')
legend('ddq1','ddq2','ddq3','Location','best')

% % %% Simulink signals
% % 
% % ss_tout = out.tout;
% % 
% % ss_q1 = out.ss_q1;
% % ss_q2 = out.ss_q2;
% % ss_q3 = out.ss_q3;
% % 
% % ss_dq1 = out.ss_dq1;
% % ss_dq2 = out.ss_dq2;
% % ss_dq3 = out.ss_dq3;
% % 
% % ss_ddq1 = out.ss_ddq1;
% % ss_ddq2 = out.ss_ddq2;
% % ss_ddq3 = out.ss_ddq3;
% % 
% % ss_tau1 = out.ss_tau1;
% % ss_tau2 = out.ss_tau2;
% % ss_tau3 = out.ss_tau3;
% % 
% % figure
% % 
% % %% Joint Position
% % 
% % subplot(4,1,1)
% % plot(an_tout,an_q1,'b','LineWidth',1.5)
% % hold on
% % plot(out.tout,out.ss_q1,'r--','LineWidth',1.5)
% % grid on
% % xlabel('Time (s)')
% % ylabel('q_1 (rad)')
% % title('Joint 1 Position')
% % legend('Analytical','Simulink','Location','best')
% % 
% % %% Joint Velocity
% % 
% % subplot(4,1,2)
% % plot(an_tout,an_dq1,'b','LineWidth',1.5)
% % hold on
% % plot(out.tout,out.ss_dq1,'r--','LineWidth',1.5)
% % grid on
% % xlabel('Time (s)')
% % ylabel('dq_1 (rad/s)')
% % title('Joint 1 Velocity')
% % legend('Analytical','Simulink','Location','best')
% % 
% % %% Joint Acceleration
% % 
% % subplot(4,1,3)
% % plot(an_tout,an_ddq1,'b','LineWidth',1.5)
% % hold on
% % plot(out.tout,out.ss_ddq1,'r--','LineWidth',1.5)
% % grid on
% % xlabel('Time (s)')
% % ylabel('ddq_1 (rad/s^2)')
% % title('Joint 1 Acceleration')
% % legend('Analytical','Simulink','Location','best')
% % 
% % %% Joint Torque
% % 
% % subplot(4,1,4)
% % plot(an_tout,an_tau1,'b','LineWidth',1.5)
% % hold on
% % plot(out.tout,out.ss_tau1,'r--','LineWidth',1.5)
% % grid on
% % xlabel('Time (s)')
% % ylabel('Torque (Nm)')
% % title('Joint 1 Torque')
% % legend('Analytical','Simulink','Location','best')



% %Average the analytical data
% windowSize = floor(length(an_tout)/1000);
% 
% numWindows = floor(length(an_tout)/windowSize);
% 
% an_t_avg   = zeros(numWindows,1);
% an_q1_avg  = zeros(numWindows,1);
% an_dq1_avg = zeros(numWindows,1);
% an_ddq1_avg= zeros(numWindows,1);
% an_tau1_avg= zeros(numWindows,1);
% 
% for k = 1:numWindows
% 
%     idx = (k-1)*windowSize + 1 : k*windowSize;
% 
%     an_t_avg(k)    = mean(an_tout(idx));
%     an_q1_avg(k)   = mean(an_q1(idx));
%     an_dq1_avg(k)  = mean(an_dq1(idx));
%     an_ddq1_avg(k) = mean(an_ddq1(idx));
%     an_tau1_avg(k) = mean(an_tau1(idx));
% 
% end
% 
% %Average the Simulink data using the same number of bins
% windowSize = floor(length(out.tout)/1000);
% 
% numWindows = floor(length(out.tout)/windowSize);
% 
% ss_t_avg   = zeros(numWindows,1);
% ss_q1_avg  = zeros(numWindows,1);
% ss_dq1_avg = zeros(numWindows,1);
% ss_ddq1_avg= zeros(numWindows,1);
% ss_tau1_avg= zeros(numWindows,1);
% 
% for k = 1:numWindows
% 
%     idx = (k-1)*windowSize + 1 : k*windowSize;
% 
%     ss_t_avg(k)    = mean(out.tout(idx));
%     ss_q1_avg(k)   = mean(out.ss_q1(idx));
%     ss_dq1_avg(k)  = mean(out.ss_dq1(idx));
%     ss_ddq1_avg(k) = mean(out.ss_ddq1(idx));
%     ss_tau1_avg(k) = mean(out.ss_tau1(idx));
% 
% end
% 
% figure
% 
% subplot(4,1,1)
% plot(an_t_avg,an_q1_avg,'b','LineWidth',1.5)
% hold on
% plot(ss_t_avg,ss_q1_avg,'r--','LineWidth',1.5)
% grid on
% title('Joint 1 Position')
% legend('Analytical','Simulink')
% 
% subplot(4,1,2)
% plot(an_t_avg,an_dq1_avg,'b','LineWidth',1.5)
% hold on
% plot(ss_t_avg,ss_dq1_avg,'r--','LineWidth',1.5)
% grid on
% title('Joint 1 Velocity')
% 
% subplot(4,1,3)
% plot(an_t_avg,an_ddq1_avg,'b','LineWidth',1.5)
% hold on
% plot(ss_t_avg,ss_ddq1_avg,'r--','LineWidth',1.5)
% grid on
% title('Joint 1 Acceleration')
% 
% subplot(4,1,4)
% plot(an_t_avg,an_tau1_avg,'b','LineWidth',1.5)
% hold on
% plot(ss_t_avg,ss_tau1_avg,'r--','LineWidth',1.5)
% grid on
% title('Joint 1 Torque')

% %errors
% error_q1   = an_q1_avg   - ss_q1_avg;
% error_dq1  = an_dq1_avg  - ss_dq1_avg;
% error_ddq1 = an_ddq1_avg - ss_ddq1_avg;
% error_tau1 = an_tau1_avg - ss_tau1_avg;
% 
% RMSE = [ ...
%     sqrt(mean(error_q1.^2));
%     sqrt(mean(error_dq1.^2));
%     sqrt(mean(error_ddq1.^2));
%     sqrt(mean(error_tau1.^2))];
% 
% MAE = [ ...
%     mean(abs(error_q1));
%     mean(abs(error_dq1));
%     mean(abs(error_ddq1));
%     mean(abs(error_tau1))];
% 
% MaxError = [ ...
%     max(abs(error_q1));
%     max(abs(error_dq1));
%     max(abs(error_ddq1));
%     max(abs(error_tau1))];
% 
% MeanError = [ ...
%     mean(error_q1);
%     mean(error_dq1);
%     mean(error_ddq1);
%     mean(error_tau1)];
% 
% Joint1_ErrorTable = table( ...
%     ["Position";"Velocity";"Acceleration";"Torque"], ...
%     RMSE,...
%     MAE,...
%     MaxError,...
%     MeanError,...
%     'VariableNames',{'Signal','RMSE','MAE','MaxError','MeanError'});
% 
% disp(Joint1_ErrorTable)
% 
% 
% 
% figure
% 
% subplot(4,1,1)
% plot(an_t_avg,error_q1,'k','LineWidth',1.5)
% grid on
% title('Position Error')
% ylabel('rad')
% 
% subplot(4,1,2)
% plot(an_t_avg,error_dq1,'k','LineWidth',1.5)
% grid on
% title('Velocity Error')
% ylabel('rad/s')
% 
% subplot(4,1,3)
% plot(an_t_avg,error_ddq1,'k','LineWidth',1.5)
% grid on
% title('Acceleration Error')
% ylabel('rad/s^2')
% 
% subplot(4,1,4)
% plot(an_t_avg,error_tau1,'k','LineWidth',1.5)
% grid on
% title('Torque Error')
% ylabel('Nm')
% xlabel('Time (s)')