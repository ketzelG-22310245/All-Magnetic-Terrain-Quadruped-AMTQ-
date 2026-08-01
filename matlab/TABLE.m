%% ============================================================
% Statistical Analysis of Joint Angular Velocities
% Adapted DAR Method - AMTQ
% ============================================================

%---------------------------------------------------------------
% Datos
%---------------------------------------------------------------
Joint = {'q_1';'q_2';'q_3'};

MaxVel  = [max(dq1); max(dq2); max(dq3)];
MinVel  = [min(dq1); min(dq2); min(dq3)];
MeanVel = [mean(dq1); mean(dq2); mean(dq3)];
RMSVel  = [rms(dq1); rms(dq2); rms(dq3)];
StdVel  = [std(dq1); std(dq2); std(dq3)];

Results = table(Joint,MaxVel,MinVel,MeanVel,RMSVel,StdVel);

% Redondear a 3 decimales
Results.MaxVel  = round(Results.MaxVel,3);
Results.MinVel  = round(Results.MinVel,3);
Results.MeanVel = round(Results.MeanVel,3);
Results.RMSVel  = round(Results.RMSVel,3);
Results.StdVel  = round(Results.StdVel,3);

%---------------------------------------------------------------
% Crear figura
%---------------------------------------------------------------

f = figure('Color','w',...
           'Position',[300 300 900 220],...
           'Name','Joint Velocity Statistics');

axis off

%---------------------------------------------------------------
% Crear tabla
%---------------------------------------------------------------

uit = uitable(...
    'Parent',f,...
    'Data',table2cell(Results),...
    'ColumnName',{'Joint','Maximum','Minimum','Mean','RMS','Std Dev'},...
    'Units','normalized',...
    'Position',[0.02 0.02 0.96 0.82],...
    'FontSize',12);

%---------------------------------------------------------------
% Título
%---------------------------------------------------------------

annotation(f,'textbox',...
    [0.02 0.86 0.96 0.12],...
    'String','Statistical Analysis of Joint Angular Velocities using the Adapted DAR Method',...
    'EdgeColor','none',...
    'HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',14);

%---------------------------------------------------------------
% Guardar automáticamente
%---------------------------------------------------------------

exportgraphics(f,...
    'JointVelocityStatistics.png',...
    'Resolution',300);

% También guardar en Excel
writetable(Results,'JointVelocityStatistics.xlsx');

disp('------------------------------------------------');
disp('Table exported successfully.');
disp('PNG : JointVelocityStatistics.png');
disp('Excel: JointVelocityStatistics.xlsx');
disp('------------------------------------------------');