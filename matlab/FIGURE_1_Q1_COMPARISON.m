%% ============================================================
% FIGURE 1
% Joint Angle Comparison - Coxa
% Classical IK vs DAR
%==============================================================

clearvars -except outIK outDAR
close all
clc

%% ============================================================
% Time
%==============================================================

t = outIK.tout;

%% ============================================================
% Joint q1 (Leg 1)
%==============================================================

q1_IK  = squeeze(outIK.IK_Q1(1,1,:));
q1_DAR = squeeze(outDAR.DAR_Q1(1,1,:));

%% ============================================================
% Plot
%==============================================================

figure('Color','w','Position',[120 100 1000 500])

plot(t,q1_IK,'b','LineWidth',2)
hold on

plot(t,q1_DAR,'r--','LineWidth',2)

grid on
box on

xlabel('Time (s)','FontSize',13)
ylabel('Joint Angle (rad)','FontSize',13)

title('Coxa Joint Angle Comparison','FontSize',15)

legend('Classical IK',...
       'DAR',...
       'Location','best')

set(gca,'FontSize',12)

%% ============================================================
% Metrics
%==============================================================

Range_IK = max(q1_IK)-min(q1_IK);
Range_DAR = max(q1_DAR)-min(q1_DAR);

Mean_IK = mean(q1_IK);
Mean_DAR = mean(q1_DAR);

RMS_IK = rms(q1_IK);
RMS_DAR = rms(q1_DAR);

STD_IK = std(q1_IK);
STD_DAR = std(q1_DAR);

Results = table(...
    ["Range (rad)";
     "Mean (rad)";
     "RMS (rad)";
     "Std Dev (rad)"],...
    [Range_IK;
     Mean_IK;
     RMS_IK;
     STD_IK],...
    [Range_DAR;
     Mean_DAR;
     RMS_DAR;
     STD_DAR],...
    'VariableNames',{'Metric','IK','DAR'});

disp(' ')
disp('=========== Q1 COMPARISON ===========')
disp(Results)