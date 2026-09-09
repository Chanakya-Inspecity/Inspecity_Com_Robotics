% clc;
% 
% %% Extract Simscape outputs from sim result
% ss_tout = out.tout;
% 
% ss_q1   = out.ss_q1;   ss_q2   = out.ss_q2;   ss_q3   = out.ss_q3;
% ss_dq1  = out.ss_dq1;  ss_dq2  = out.ss_dq2;  ss_dq3  = out.ss_dq3;
% ss_ddq1 = out.ss_ddq1; ss_ddq2 = out.ss_ddq2; ss_ddq3 = out.ss_ddq3;
% ss_tau1 = out.ss_tau1; ss_tau2 = out.ss_tau2; ss_tau3 = out.ss_tau3;
% 
% %% ==========================================================
% %  Interpolation-based comparison
% %  Both signals resampled to a common 1 ms grid via PCHIP.
% %  This correctly handles the fact that the NE model outputs
% %  at fixed dt=1e-4 while Simscape uses a variable-step solver
% %  (so the kth sample in each model occurs at different times).
% %% ==========================================================
% 
% t_common = (0 : 1e-3 : 20)';
% 
% % --- Interpolate NE (analytical) ---
% ne_q1i   = interp1(an_tout, an_q1,   t_common, 'pchip');
% ne_q2i   = interp1(an_tout, an_q2,   t_common, 'pchip');
% ne_q3i   = interp1(an_tout, an_q3,   t_common, 'pchip');
% ne_dq1i  = interp1(an_tout, an_dq1,  t_common, 'pchip');
% ne_dq2i  = interp1(an_tout, an_dq2,  t_common, 'pchip');
% ne_dq3i  = interp1(an_tout, an_dq3,  t_common, 'pchip');
% ne_ddq1i = interp1(an_tout, an_ddq1, t_common, 'pchip');
% ne_ddq2i = interp1(an_tout, an_ddq2, t_common, 'pchip');
% ne_ddq3i = interp1(an_tout, an_ddq3, t_common, 'pchip');
% ne_tau1i = interp1(an_tout, an_tau1, t_common, 'pchip');
% ne_tau2i = interp1(an_tout, an_tau2, t_common, 'pchip');
% ne_tau3i = interp1(an_tout, an_tau3, t_common, 'pchip');
% 
% % --- Interpolate Simscape ---
% sc_q1i   = interp1(ss_tout, ss_q1,   t_common, 'pchip');
% sc_q2i   = interp1(ss_tout, ss_q2,   t_common, 'pchip');
% sc_q3i   = interp1(ss_tout, ss_q3,   t_common, 'pchip');
% sc_dq1i  = interp1(ss_tout, ss_dq1,  t_common, 'pchip');
% sc_dq2i  = interp1(ss_tout, ss_dq2,  t_common, 'pchip');
% sc_dq3i  = interp1(ss_tout, ss_dq3,  t_common, 'pchip');
% sc_ddq1i = interp1(ss_tout, ss_ddq1, t_common, 'pchip');
% sc_ddq2i = interp1(ss_tout, ss_ddq2, t_common, 'pchip');
% sc_ddq3i = interp1(ss_tout, ss_ddq3, t_common, 'pchip');
% sc_tau1i = interp1(ss_tout, ss_tau1, t_common, 'pchip');
% sc_tau2i = interp1(ss_tout, ss_tau2, t_common, 'pchip');
% sc_tau3i = interp1(ss_tout, ss_tau3, t_common, 'pchip');
% 
% %% ==========================================================
% %  Point-by-point errors on the common grid
% %% ==========================================================
% 
% error_q1   = ne_q1i   - sc_q1i;    error_q2   = ne_q2i   - sc_q2i;    error_q3   = ne_q3i   - sc_q3i;
% error_dq1  = ne_dq1i  - sc_dq1i;   error_dq2  = ne_dq2i  - sc_dq2i;   error_dq3  = ne_dq3i  - sc_dq3i;
% error_ddq1 = ne_ddq1i - sc_ddq1i;  error_ddq2 = ne_ddq2i - sc_ddq2i;  error_ddq3 = ne_ddq3i - sc_ddq3i;
% error_tau1 = ne_tau1i - sc_tau1i;  error_tau2 = ne_tau2i - sc_tau2i;  error_tau3 = ne_tau3i - sc_tau3i;
% 
% %% ==========================================================
% %  Error tables
% %% ==========================================================
% 
% function T = makeErrorTable(e_q, e_dq, e_ddq, e_tau)
%     signals = ["Position"; "Velocity"; "Acceleration"; "Torque"];
%     errors  = [e_q, e_dq, e_ddq, e_tau];
%     RMSE  = sqrt(mean(errors.^2))';
%     MAE   = mean(abs(errors))';
%     MaxE  = max(abs(errors))';
%     MeanE = mean(errors)';
%     T = table(signals, RMSE, MAE, MaxE, MeanE, ...
%         'VariableNames', {'Signal','RMSE','MAE','MaxError','MeanError'});
% end
% 
% disp(' ')
% disp('========== Joint 1 Error Table ==========')
% disp(makeErrorTable(error_q1, error_dq1, error_ddq1, error_tau1))
% 
% disp(' ')
% disp('========== Joint 2 Error Table ==========')
% disp(makeErrorTable(error_q2, error_dq2, error_ddq2, error_tau2))
% 
% disp(' ')
% disp('========== Joint 3 Error Table ==========')
% disp(makeErrorTable(error_q3, error_dq3, error_ddq3, error_tau3))
% 
% %% ==========================================================
% %  Comparison plots + error plots
% %% ==========================================================
% 
% function plotValidationComparison( ...
%     t_ne, t_ss, t_err, ...
%     ne_signal, ss_signal, ...
%     signal_error, ...
%     jointNumber, signalName, signalUnit)
% 
% figure('Name', sprintf('Joint %d - %s Validation', jointNumber, signalName), ...
%        'Color','w', 'Position',[100 100 900 600]);
% 
% subplot(2,1,1)
% plot(t_ne, ne_signal, 'b',  'LineWidth', 1.8); hold on
% plot(t_ss, ss_signal, 'r--','LineWidth', 1.5)
% grid on; box on
% xlabel('Time (s)'); ylabel(signalUnit)
% title(sprintf('Joint %d  %s', jointNumber, signalName))
% legend('NE Analytical','Simscape','Location','best')
% 
% subplot(2,1,2)
% plot(t_err, signal_error, 'k', 'LineWidth', 1.5)
% grid on; box on
% xlabel('Time (s)'); ylabel(['Error (' signalUnit ')'])
% title(sprintf('Joint %d  %s Error  (RMSE = %.3e %s)', ...
%     jointNumber, signalName, sqrt(mean(signal_error.^2)), signalUnit))
% 
% outputFolder = 'Validation_Plots3_with_g0';
% if ~exist(outputFolder,'dir'), mkdir(outputFolder); end
% fileName = sprintf('Joint%d_%s', jointNumber, signalName);
% saveas(gcf, fullfile(outputFolder, [fileName '.png']));
% savefig(gcf, fullfile(outputFolder, [fileName '.fig']));
% end
% 
% %% -------------------------------
% %  Joint 1
% %% -------------------------------
% 
% plotValidationComparison(an_tout, ss_tout, t_common, ...
%     an_q1,   ss_q1,   error_q1,   1, 'Position',     'rad');
% plotValidationComparison(an_tout, ss_tout, t_common, ...
%     an_dq1,  ss_dq1,  error_dq1,  1, 'Velocity',     'rad/s');
% plotValidationComparison(an_tout, ss_tout, t_common, ...
%     an_ddq1, ss_ddq1, error_ddq1, 1, 'Acceleration', 'rad/s^2');
% plotValidationComparison(an_tout, ss_tout, t_common, ...
%     an_tau1, ss_tau1, error_tau1, 1, 'Torque',       'Nm');
% 
% %% -------------------------------
% %  Joint 2
% %% -------------------------------
% 
% plotValidationComparison(an_tout, ss_tout, t_common, ...
%     an_q2,   ss_q2,   error_q2,   2, 'Position',     'rad');
% plotValidationComparison(an_tout, ss_tout, t_common, ...
%     an_dq2,  ss_dq2,  error_dq2,  2, 'Velocity',     'rad/s');
% plotValidationComparison(an_tout, ss_tout, t_common, ...
%     an_ddq2, ss_ddq2, error_ddq2, 2, 'Acceleration', 'rad/s^2');
% plotValidationComparison(an_tout, ss_tout, t_common, ...
%     an_tau2, ss_tau2, error_tau2, 2, 'Torque',       'Nm');
% 
% %% -------------------------------
% %  Joint 3
% %% -------------------------------
% 
% plotValidationComparison(an_tout, ss_tout, t_common, ...
%     an_q3,   ss_q3,   error_q3,   3, 'Position',     'rad');
% plotValidationComparison(an_tout, ss_tout, t_common, ...
%     an_dq3,  ss_dq3,  error_dq3,  3, 'Velocity',     'rad/s');
% plotValidationComparison(an_tout, ss_tout, t_common, ...
%     an_ddq3, ss_ddq3, error_ddq3, 3, 'Acceleration', 'rad/s^2');
% plotValidationComparison(an_tout, ss_tout, t_common, ...
%     an_tau3, ss_tau3, error_tau3, 3, 'Torque',       'Nm');









%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% with avg %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clc;

%% Simulink signals

ss_tout = out.tout;

ss_q1 = out.ss_q1;
ss_q2 = out.ss_q2;
ss_q3 = out.ss_q3;

ss_dq1 = out.ss_dq1;
ss_dq2 = out.ss_dq2;
ss_dq3 = out.ss_dq3;

ss_ddq1 = out.ss_ddq1;
ss_ddq2 = out.ss_ddq2;
ss_ddq3 = out.ss_ddq3;

ss_tau1 = out.ss_tau1;
ss_tau2 = out.ss_tau2;
ss_tau3 = out.ss_tau3;


%% ==========================================================
% Average Analytical Data
%% ==========================================================

numBins = 400;%%50 smaple size

windowSize = floor(length(an_tout)/numBins);
numWindows = floor(length(an_tout)/windowSize);

an_t_avg = zeros(numWindows,1);

an_q1_avg   = zeros(numWindows,1);
an_q2_avg   = zeros(numWindows,1);
an_q3_avg   = zeros(numWindows,1);

an_dq1_avg  = zeros(numWindows,1);
an_dq2_avg  = zeros(numWindows,1);
an_dq3_avg  = zeros(numWindows,1);

an_ddq1_avg = zeros(numWindows,1);
an_ddq2_avg = zeros(numWindows,1);
an_ddq3_avg = zeros(numWindows,1);

an_tau1_avg = zeros(numWindows,1);
an_tau2_avg = zeros(numWindows,1);
an_tau3_avg = zeros(numWindows,1);

for k = 1:numWindows

    idx = (k-1)*windowSize+1 : k*windowSize;

    an_t_avg(k) = mean(an_tout(idx));

    % Position
    an_q1_avg(k) = mean(an_q1(idx));
    an_q2_avg(k) = mean(an_q2(idx));
    an_q3_avg(k) = mean(an_q3(idx));

    % Velocity
    an_dq1_avg(k) = mean(an_dq1(idx));
    an_dq2_avg(k) = mean(an_dq2(idx));
    an_dq3_avg(k) = mean(an_dq3(idx));

    % Acceleration
    an_ddq1_avg(k) = mean(an_ddq1(idx));
    an_ddq2_avg(k) = mean(an_ddq2(idx));
    an_ddq3_avg(k) = mean(an_ddq3(idx));

    % Torque
    an_tau1_avg(k) = mean(an_tau1(idx));
    an_tau2_avg(k) = mean(an_tau2(idx));
    an_tau3_avg(k) = mean(an_tau3(idx));

end

%% ==========================================================
% Average Simulink Data
%% ==========================================================

windowSize = floor(length(out.tout)/numBins);
numWindows = floor(length(out.tout)/windowSize);

ss_t_avg = zeros(numWindows,1);

ss_q1_avg   = zeros(numWindows,1);
ss_q2_avg   = zeros(numWindows,1);
ss_q3_avg   = zeros(numWindows,1);

ss_dq1_avg  = zeros(numWindows,1);
ss_dq2_avg  = zeros(numWindows,1);
ss_dq3_avg  = zeros(numWindows,1);

ss_ddq1_avg = zeros(numWindows,1);
ss_ddq2_avg = zeros(numWindows,1);
ss_ddq3_avg = zeros(numWindows,1);

ss_tau1_avg = zeros(numWindows,1);
ss_tau2_avg = zeros(numWindows,1);
ss_tau3_avg = zeros(numWindows,1);

for k = 1:numWindows

    idx = (k-1)*windowSize+1 : k*windowSize;

    ss_t_avg(k) = mean(out.tout(idx));

    % Position
    ss_q1_avg(k) = mean(out.ss_q1(idx));
    ss_q2_avg(k) = mean(out.ss_q2(idx));
    ss_q3_avg(k) = mean(out.ss_q3(idx));

    % Velocity
    ss_dq1_avg(k) = mean(out.ss_dq1(idx));
    ss_dq2_avg(k) = mean(out.ss_dq2(idx));
    ss_dq3_avg(k) = mean(out.ss_dq3(idx));

    % Acceleration
    ss_ddq1_avg(k) = mean(out.ss_ddq1(idx));
    ss_ddq2_avg(k) = mean(out.ss_ddq2(idx));
    ss_ddq3_avg(k) = mean(out.ss_ddq3(idx));

    % Torque
    ss_tau1_avg(k) = mean(out.ss_tau1(idx));
    ss_tau2_avg(k) = mean(out.ss_tau2(idx));
    ss_tau3_avg(k) = mean(out.ss_tau3(idx));

end

%% ==========================================================
% Joint 1 Errors
%% ==========================================================

error_q1   = an_q1_avg   - ss_q1_avg;
error_dq1  = an_dq1_avg  - ss_dq1_avg;
error_ddq1 = an_ddq1_avg - ss_ddq1_avg;
error_tau1 = an_tau1_avg - ss_tau1_avg;

Joint1_ErrorTable = table( ...
    ["Position";"Velocity";"Acceleration";"Torque"], ...
    [sqrt(mean(error_q1.^2));
     sqrt(mean(error_dq1.^2));
     sqrt(mean(error_ddq1.^2));
     sqrt(mean(error_tau1.^2))], ...
    [mean(abs(error_q1));
     mean(abs(error_dq1));
     mean(abs(error_ddq1));
     mean(abs(error_tau1))], ...
    [max(abs(error_q1));
     max(abs(error_dq1));
     max(abs(error_ddq1));
     max(abs(error_tau1))], ...
    [mean(error_q1);
     mean(error_dq1);
     mean(error_ddq1);
     mean(error_tau1)], ...
    'VariableNames',{'Signal','RMSE','MAE','MaxError','MeanError'});

disp(' ')
disp('========== Joint 1 Error Table ==========')
disp(Joint1_ErrorTable)


%% ==========================================================
% Joint 2 Errors
%% ==========================================================

error_q2   = an_q2_avg   - ss_q2_avg;
error_dq2  = an_dq2_avg  - ss_dq2_avg;
error_ddq2 = an_ddq2_avg - ss_ddq2_avg;
error_tau2 = an_tau2_avg - ss_tau2_avg;

Joint2_ErrorTable = table( ...
    ["Position";"Velocity";"Acceleration";"Torque"], ...
    [sqrt(mean(error_q2.^2));
     sqrt(mean(error_dq2.^2));
     sqrt(mean(error_ddq2.^2));
     sqrt(mean(error_tau2.^2))], ...
    [mean(abs(error_q2));
     mean(abs(error_dq2));
     mean(abs(error_ddq2));
     mean(abs(error_tau2))], ...
    [max(abs(error_q2));
     max(abs(error_dq2));
     max(abs(error_ddq2));
     max(abs(error_tau2))], ...
    [mean(error_q2);
     mean(error_dq2);
     mean(error_ddq2);
     mean(error_tau2)], ...
    'VariableNames',{'Signal','RMSE','MAE','MaxError','MeanError'});

disp(' ')
disp('========== Joint 2 Error Table ==========')
disp(Joint2_ErrorTable)


%% ==========================================================
% Joint 3 Errors
%% ==========================================================

error_q3   = an_q3_avg   - ss_q3_avg;
error_dq3  = an_dq3_avg  - ss_dq3_avg;
error_ddq3 = an_ddq3_avg - ss_ddq3_avg;
error_tau3 = an_tau3_avg - ss_tau3_avg;

Joint3_ErrorTable = table( ...
    ["Position";"Velocity";"Acceleration";"Torque"], ...
    [sqrt(mean(error_q3.^2));
     sqrt(mean(error_dq3.^2));
     sqrt(mean(error_ddq3.^2));
     sqrt(mean(error_tau3.^2))], ...
    [mean(abs(error_q3));
     mean(abs(error_dq3));
     mean(abs(error_ddq3));
     mean(abs(error_tau3))], ...
    [max(abs(error_q3));
     max(abs(error_dq3));
     max(abs(error_ddq3));
     max(abs(error_tau3))], ...
    [mean(error_q3);
     mean(error_dq3);
     mean(error_ddq3);
     mean(error_tau3)], ...
    'VariableNames',{'Signal','RMSE','MAE','MaxError','MeanError'});

disp(' ')
disp('========== Joint 3 Error Table ==========')
disp(Joint3_ErrorTable)




function plotValidationComparison( ...
    an_t, ss_t, ...
    an_signal, ss_signal, ...
    signal_error, ...
    jointNumber, ...
    signalName, ...
    signalUnit)

%==========================================================
% Plots Analytical vs Simulink comparison and error
%
% Inputs:
% an_t          : Analytical time
% ss_t          : Simulink time
% an_signal     : Analytical signal
% ss_signal     : Simulink signal
% signal_error  : an_signal - ss_signal
% jointNumber   : Joint number (1,2,3)
% signalName    : 'Position','Velocity',...
% signalUnit    : 'rad','rad/s',...
%
% Example:
%
% plotValidationComparison(...
%     an_t_avg,ss_t_avg,...
%     an_q1_avg,ss_q1_avg,...
%     error_q1,...
%     1,...
%     'Position',...
%     'rad');
%
%==========================================================

figure( ...
    'Name',sprintf('Joint %d - %s Validation',jointNumber,signalName), ...
    'Color','w');

%% ===============================================
%% Comparison Plot
%% ===============================================

subplot(2,1,1)

plot(an_t,an_signal,...
    'b',...
    'LineWidth',2);

hold on

plot(ss_t,ss_signal,...
    'r--',...
    'LineWidth',2);

grid on
box on

xlabel('Time (s)')
ylabel(signalUnit)

title(sprintf('Joint %d %s',jointNumber,signalName))

legend( ...
    'Analytical',...
    'Simulink',...
    'Location','best')

%% ===============================================
%% Error Plot
%% ===============================================

subplot(2,1,2)

plot(an_t,...
    signal_error,...
    'k',...
    'LineWidth',1.8);

grid on
box on

xlabel('Time (s)')
ylabel(['Error (' signalUnit ')'])

title(sprintf('Joint %d %s Error',jointNumber,signalName))


%% Save Figure

outputFolder = 'Validation_Plots3_with_g';

if ~exist(outputFolder,'dir')
    mkdir(outputFolder);
end

fileName = sprintf('Joint%d_%s',jointNumber,signalName);

saveas(gcf, fullfile(outputFolder,[fileName '.png']));
savefig(gcf, fullfile(outputFolder,[fileName '.fig']));
end

%% -------------------------------
%% Joint 1
%% -------------------------------

plotValidationComparison( ...
    an_t_avg, ss_t_avg, ...
    an_q1_avg, ss_q1_avg, ...
    error_q1, ...
    1, ...
    'Position', ...
    'rad');

plotValidationComparison( ...
    an_t_avg, ss_t_avg, ...
    an_dq1_avg, ss_dq1_avg, ...
    error_dq1, ...
    1, ...
    'Velocity', ...
    'rad/s');

plotValidationComparison( ...
    an_t_avg, ss_t_avg, ...
    an_ddq1_avg, ss_ddq1_avg, ...
    error_ddq1, ...
    1, ...
    'Acceleration', ...
    'rad/s^2');

plotValidationComparison( ...
    an_t_avg, ss_t_avg, ...
    an_tau1_avg, ss_tau1_avg, ...
    error_tau1, ...
    1, ...
    'Torque', ...
    'Nm');


%% -------------------------------
%% Joint 2
%% -------------------------------

plotValidationComparison( ...
    an_t_avg, ss_t_avg, ...
    an_q2_avg, ss_q2_avg, ...
    error_q2, ...
    2, ...
    'Position', ...
    'rad');

plotValidationComparison( ...
    an_t_avg, ss_t_avg, ...
    an_dq2_avg, ss_dq2_avg, ...
    error_dq2, ...
    2, ...
    'Velocity', ...
    'rad/s');

plotValidationComparison( ...
    an_t_avg, ss_t_avg, ...
    an_ddq2_avg, ss_ddq2_avg, ...
    error_ddq2, ...
    2, ...
    'Acceleration', ...
    'rad/s^2');

plotValidationComparison( ...
    an_t_avg, ss_t_avg, ...
    an_tau2_avg, ss_tau2_avg, ...
    error_tau2, ...
    2, ...
    'Torque', ...
    'Nm');


%% -------------------------------
%% Joint 3
%% -------------------------------

plotValidationComparison( ...
    an_t_avg, ss_t_avg, ...
    an_q3_avg, ss_q3_avg, ...
    error_q3, ...
    3, ...
    'Position', ...
    'rad');

plotValidationComparison( ...
    an_t_avg, ss_t_avg, ...
    an_dq3_avg, ss_dq3_avg, ...
    error_dq3, ...
    3, ...
    'Velocity', ...
    'rad/s');

plotValidationComparison( ...
    an_t_avg, ss_t_avg, ...
    an_ddq3_avg, ss_ddq3_avg, ...
    error_ddq3, ...
    3, ...
    'Acceleration', ...
    'rad/s^2');

plotValidationComparison( ...
    an_t_avg, ss_t_avg, ...
    an_tau3_avg, ss_tau3_avg, ...
    error_tau3, ...
    3, ...
    'Torque', ...
    'Nm');

