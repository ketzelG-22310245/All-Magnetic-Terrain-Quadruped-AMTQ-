%% ==============================================================
% FIGURE 1
% COMPARISON OF CARTESIAN FOOT TRAJECTORY
% AMTQ V1 vs AMTQ V2 (DAR)
%
% Thesis:
% Adaptation of the Directed Angular Restitution (DAR) Method
% to the AMTQ Quadruped Robot
% ===============================================================

clearvars -except XYZ_REAL_V1 XYZ_REAL_V2

close all
clc

%% ===============================================================
% Extract Transform Sensor Data
% ===============================================================

V1 = squeeze(XYZ_REAL_V1);
V2 = squeeze(XYZ_REAL_V2);

X1 = V1(1,:)*1000;
Y1 = V1(2,:)*1000;
Z1 = V1(3,:)*1000;

X2 = V2(1,:)*1000;
Y2 = V2(2,:)*1000;
Z2 = V2(3,:)*1000;

%% ===============================================================
% Kinematic Metrics
% ===============================================================

StepLength_V1 = max(X1)-min(X1);
StepLength_V2 = max(X2)-min(X2);

StepHeight_V1 = max(Z1)-min(Z1);
StepHeight_V2 = max(Z2)-min(Z2);

LateralMotion_V1 = max(Y1)-min(Y1);
LateralMotion_V2 = max(Y2)-min(Y2);

%% ===============================================================
% Improvement
% ===============================================================

Improvement_Length = ...
((StepLength_V2-StepLength_V1)/StepLength_V1)*100;

Improvement_Height = ...
((StepHeight_V2-StepHeight_V1)/StepHeight_V1)*100;

Improvement_Lateral = ...
((LateralMotion_V2-LateralMotion_V1)/LateralMotion_V1)*100;

%% ===============================================================
% Figure
% ===============================================================

figure('Color','w',...
       'Position',[100 100 1100 700])

plot(X1,Z1,...
    '--',...
    'LineWidth',2,...
    'Color',[0.85 0.25 0.25])

hold on

plot(X2,Z2,...
    '-',...
    'LineWidth',2.8,...
    'Color',[0 0.35 0.85])

grid on
box on
axis equal

xlabel('X Position [mm]','FontSize',13)
ylabel('Z Position [mm]','FontSize',13)

title('Cartesian Foot Trajectory Comparison',...
      'FontSize',15,...
      'FontWeight','bold')

legend({'AMTQ V1 (Classical IK)',...
        'AMTQ V2 (DAR Method)'},...
        'Location','best')

set(gca,...
    'FontSize',12,...
    'LineWidth',1.2)

%% ===============================================================
% Results Table
% ===============================================================

Parameter = {

'Step Length (mm)'

'Step Height (mm)'

'Lateral Motion (mm)'

};

AMTQ_V1 = [

StepLength_V1

StepHeight_V1

LateralMotion_V1

];

AMTQ_V2 = [

StepLength_V2

StepHeight_V2

LateralMotion_V2

];

Improvement = [

Improvement_Length

Improvement_Height

Improvement_Lateral

];

Results = table(Parameter,...
                AMTQ_V1,...
                AMTQ_V2,...
                Improvement);

disp(' ')
disp('================= TRAJECTORY COMPARISON =================')
disp(Results)

%% ===============================================================
% Publication Table
% ===============================================================

figure('Color','w',...
       'Position',[300 300 900 180])

uitable(...
'Data',table2cell(Results),...
'ColumnName',{'Parameter','AMTQ V1','AMTQ V2','Improvement (%)'},...
'RowName',[],...
'Units','normalized',...
'Position',[0 0 1 1],...
'FontSize',12);

%% ===============================================================
% Save Figures (300 dpi)
% ===============================================================

exportgraphics(gcf,...
'Figure1_Trajectory_Table.png',...
'Resolution',300);