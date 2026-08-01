%% ============================================================
% Cartesian Tracking Error
% Adapted DAR Method
% AMTQ Quadruped
% ============================================================

t = out.tout;

%%------------------------------------------------------------
% Trayectoria deseada (Pata delantera derecha)
%%------------------------------------------------------------

Xd = squeeze(out.X_TRAJ(1,1,:));
Yd = squeeze(out.Y_TRAJ(1,1,:));
Zd = squeeze(out.Z_TRAJ(1,1,:));

%%------------------------------------------------------------
% Trayectoria medida (Transform Sensor)
%%------------------------------------------------------------

Xr = squeeze(out.XYZ_REAL(1,1,:));
Yr = squeeze(out.XYZ_REAL(2,1,:));
Zr = squeeze(out.XYZ_REAL(3,1,:));

%%------------------------------------------------------------
% Alinear ambos sistemas de referencia
%%------------------------------------------------------------

Xd = Xd - Xd(1);
Yd = Yd - Yd(1);
Zd = -(Zd - Zd(1));

Xr = Xr - Xr(1);
Yr = Yr - Yr(1);
Zr = Zr - Zr(1);

%%------------------------------------------------------------
% Error cartesiano
%%------------------------------------------------------------

Error = sqrt( ...
        (Xd-Xr).^2 + ...
        (Yd-Yr).^2 + ...
        (Zd-Zr).^2 );

%%------------------------------------------------------------
% Estadísticas
%%------------------------------------------------------------

RMS_Error  = rms(Error);
Mean_Error = mean(Error);
Max_Error  = max(Error);
Min_Error  = min(Error);

fprintf('\n');
fprintf('=========== CARTESIAN ERROR ===========\n');
fprintf('Mean Error : %.3f mm\n',Mean_Error);
fprintf('RMS Error  : %.3f mm\n',RMS_Error);
fprintf('Maximum    : %.3f mm\n',Max_Error);
fprintf('Minimum    : %.3f mm\n',Min_Error);

%%------------------------------------------------------------
% Figura
%%------------------------------------------------------------

figure('Color','w',...
       'Position',[250 150 900 350]);

plot(t,Error,'LineWidth',2)

grid on
box on

xlabel('Time (s)')
ylabel('Cartesian Error (mm)')

title('Cartesian Tracking Error using the Adapted DAR Method')

xlim([t(1) t(end)])