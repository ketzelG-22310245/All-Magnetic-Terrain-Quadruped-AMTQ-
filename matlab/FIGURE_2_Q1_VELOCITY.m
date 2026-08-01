%% ============================================================
% FIGURE 2
% Angular Velocity Comparison (Coxa)
% Classical IK vs DAR
%==============================================================

clearvars -except outIK outDAR
close all
clc

%% ============================================================
% Time
%==============================================================

t = outIK.tout;
Ts = mean(diff(t));

%% ============================================================
% Joint Angle
%==============================================================

qIK  = squeeze(outIK.IK_Q1(1,1,:));
qDAR = squeeze(outDAR.DAR_Q1(1,1,:));

%% ============================================================
% Angular Velocity
%==============================================================

dqIK  = gradient(qIK,Ts);
dqDAR = gradient(qDAR,Ts);

%% ============================================================
% Plot
%==============================================================

figure('Color','w','Position',[100 100 1100 500])

plot(t,dqIK,'b','LineWidth',2)
hold on

plot(t,dqDAR,'r--','LineWidth',2)

grid on
box on

xlabel('Time (s)','FontSize',13)
ylabel('Angular Velocity (rad/s)','FontSize',13)

title('Coxa Angular Velocity Comparison','FontSize',15)

legend('Classical IK','DAR','Location','best')

set(gca,'FontSize',12)

%% ============================================================
% Statistics
%==============================================================

PeakIK=max(abs(dqIK));
PeakDAR=max(abs(dqDAR));

RMSIK=rms(dqIK);
RMSDAR=rms(dqDAR);

MeanIK=mean(abs(dqIK));
MeanDAR=mean(abs(dqDAR));

Results=table(...
["Peak Velocity (rad/s)";
 "Mean Velocity (rad/s)";
 "RMS Velocity (rad/s)"],...
[PeakIK;
 MeanIK;
 RMSIK],...
[PeakDAR;
 MeanDAR;
 RMSDAR],...
'VariableNames',{'Metric','IK','DAR'});

disp(' ')
disp('============== ANGULAR VELOCITY ==============')
disp(Results)