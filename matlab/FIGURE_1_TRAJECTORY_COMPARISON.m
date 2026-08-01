%% ============================================================
% FIGURE 1
% Cartesian Foot Trajectory Comparison
% Classical IK vs DAR
%==============================================================

clearvars -except outIK outDAR
close all
clc

%% ============================================================
% Desired Trajectory (Foot 1)
%==============================================================

Xd = squeeze(outIK.X_TRAJ(1,1,:));
Yd = squeeze(outIK.Y_TRAJ(1,1,:));
Zd = squeeze(outIK.Z_TRAJ(1,1,:));

%% ============================================================
% Classical IK Trajectory
% Transform Sensor -> meters
% Convert to millimeters
%==============================================================

Xik = 1000*squeeze(outIK.XYZ_REAL(1,1,:));
Yik = 1000*squeeze(outIK.XYZ_REAL(2,1,:));
Zik = 1000*squeeze(outIK.XYZ_REAL(3,1,:));

%% ============================================================
% DAR Trajectory
%==============================================================

Xdar = 1000*squeeze(outDAR.XYZ_REAL(1,1,:));
Ydar = 1000*squeeze(outDAR.XYZ_REAL(2,1,:));
Zdar = 1000*squeeze(outDAR.XYZ_REAL(3,1,:));

%% ============================================================
% Figure
%==============================================================

figure('Color','w','Position',[100 100 1000 700])

plot(Xd,Zd,'k--','LineWidth',3)
hold on

plot(Xik,Zik,'b','LineWidth',2)

plot(Xdar,Zdar,'r','LineWidth',2)

grid on
box on
axis equal

xlabel('X Position (mm)','FontSize',13)
ylabel('Z Position (mm)','FontSize',13)

title('Cartesian Foot Trajectory Comparison','FontSize',15)

legend({'Desired Trajectory',...
        'Classical IK',...
        'DAR'},...
        'Location','best')

set(gca,'FontSize',12)

%% ============================================================
% STEP METRICS
%==============================================================

StepLength_IK = max(Xik)-min(Xik);
StepLength_DAR = max(Xdar)-min(Xdar);

StepHeight_IK = max(Zik)-min(Zik);
StepHeight_DAR = max(Zdar)-min(Zdar);

Lateral_IK = max(Yik)-min(Yik);
Lateral_DAR = max(Ydar)-min(Ydar);

Parameter = {'Step Length (mm)';
             'Step Height (mm)';
             'Lateral Motion (mm)'};

IK = [StepLength_IK;
      StepHeight_IK;
      Lateral_IK];

DAR = [StepLength_DAR;
       StepHeight_DAR;
       Lateral_DAR];

Comparison = table(Parameter,IK,DAR);

disp(' ')
disp('================ TRAJECTORY METRICS ================')
disp(Comparison)

%% ============================================================
% CARTESIAN ERROR
%==============================================================

ErrorIK = sqrt((Xd-Xik).^2 + ...
               (Yd-Yik).^2 + ...
               (Zd-Zik).^2);

ErrorDAR = sqrt((Xd-Xdar).^2 + ...
                (Yd-Ydar).^2 + ...
                (Zd-Zdar).^2);

fprintf('\n================ CARTESIAN ERROR =================\n')

fprintf('\nCLASSICAL IK\n')
fprintf('Mean Error : %.3f mm\n',mean(ErrorIK))
fprintf('RMS Error  : %.3f mm\n',rms(ErrorIK))
fprintf('Maximum    : %.3f mm\n',max(ErrorIK))

fprintf('\nDAR\n')
fprintf('Mean Error : %.3f mm\n',mean(ErrorDAR))
fprintf('RMS Error  : %.3f mm\n',rms(ErrorDAR))
fprintf('Maximum    : %.3f mm\n',max(ErrorDAR))

%% ============================================================
% ERROR COMPARISON TABLE
%==============================================================

Metric = {'Mean Error (mm)';
          'RMS Error (mm)';
          'Maximum Error (mm)'};

IK_Result = [mean(ErrorIK);
             rms(ErrorIK);
             max(ErrorIK)];

DAR_Result = [mean(ErrorDAR);
              rms(ErrorDAR);
              max(ErrorDAR)];

Results = table(Metric,IK_Result,DAR_Result);

disp(' ')
disp('================ ERROR COMPARISON ================')
disp(Results)