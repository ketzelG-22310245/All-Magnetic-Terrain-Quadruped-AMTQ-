%% ============================================================
% Cartesian Velocity of the Foot
% Adapted DAR Method - AMTQ V2
% Author: Ketzel Gibran Carrillo
% ============================================================

clearvars -except out
close all
clc

%% Obtener datos de Simulink

t = out.tout;

XYZ = squeeze(out.XYZ_REAL);

X = XYZ(1,:);
Y = XYZ(2,:);
Z = XYZ(3,:);

%% ============================================================
% Suavizado de la trayectoria
% (Reduce el ruido numérico generado por Simscape)
% ============================================================

window = 31;      % Puede probar 21,31,41

Xf = smoothdata(X,'sgolay',window);
Yf = smoothdata(Y,'sgolay',window);
Zf = smoothdata(Z,'sgolay',window);

%% ============================================================
% Velocidades Cartesianas
% ============================================================

Vx = gradient(Xf,t);
Vy = gradient(Yf,t);
Vz = gradient(Zf,t);

%% ============================================================
% Estadísticas
% ============================================================

fprintf('\n=========== CARTESIAN VELOCITIES ===========\n')

fprintf('Maximum Vx : %.3f m/s\n',max(Vx))
fprintf('Minimum Vx : %.3f m/s\n',min(Vx))
fprintf('RMS Vx     : %.3f m/s\n\n',rms(Vx))

fprintf('Maximum Vy : %.3f m/s\n',max(Vy))
fprintf('Minimum Vy : %.3f m/s\n',min(Vy))
fprintf('RMS Vy     : %.3f m/s\n\n',rms(Vy))

fprintf('Maximum Vz : %.3f m/s\n',max(Vz))
fprintf('Minimum Vz : %.3f m/s\n',min(Vz))
fprintf('RMS Vz     : %.3f m/s\n',rms(Vz))

%% ============================================================
% Magnitud de la velocidad
% ============================================================

V = sqrt(Vx.^2 + Vy.^2 + Vz.^2);

%% ============================================================
% Figura principal
% ============================================================

figure('Color','w',...
       'Position',[100 100 1200 650]);

plot(t,Vx,'LineWidth',2)
hold on

plot(t,Vy,'LineWidth',2)

plot(t,Vz,'LineWidth',2)

grid on
box on

xlabel('Time (s)','FontSize',12)
ylabel('Velocity (m/s)','FontSize',12)

title({'Adapted Directed Angular Restitution (DAR)',...
       'Cartesian Velocity of the Foot'},...
       'FontWeight','bold')

legend({'V_x','V_y','V_z'},...
        'Location','best')

set(gca,'FontSize',11)

%% ============================================================
% Figura de la magnitud
% ============================================================

figure('Color','w',...
       'Position',[150 150 1100 450]);

plot(t,V,'k','LineWidth',2.2)

grid on
box on

xlabel('Time (s)','FontSize',12)

ylabel('|V| (m/s)','FontSize',12)

title({'Foot Velocity Magnitude',...
       'Adapted DAR Method'},...
       'FontWeight','bold')

set(gca,'FontSize',11)

%% ============================================================
% Tabla de estadísticas
% ============================================================

Variables = {'Vx';'Vy';'Vz'};

Maximum = [max(Vx); max(Vy); max(Vz)];
Minimum = [min(Vx); min(Vy); min(Vz)];
Mean    = [mean(Vx);mean(Vy);mean(Vz)];
RMS     = [rms(Vx); rms(Vy); rms(Vz)];
Std     = [std(Vx); std(Vy); std(Vz)];

Results = table(Variables,...
                Maximum,...
                Minimum,...
                Mean,...
                RMS,...
                Std);

disp(Results)

figure('Color','w',...
       'Position',[350 250 700 140]);

uitable('Data',table2cell(Results),...
        'ColumnName',Results.Properties.VariableNames,...
        'RowName',[],...
        'Units','Normalized',...
        'Position',[0 0 1 1]);