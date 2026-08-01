%% ==============================
% Comparación: Trayectoria Deseada vs Medida
% ==============================

%----- Trayectoria real (Simscape) -----
XYZ = squeeze(out.XYZ_REAL);

% Convertir de metros a milímetros
X_real = XYZ(1,:)*1000;
Y_real = XYZ(2,:)*1000;
Z_real = XYZ(3,:)*1000;

%----- Trayectoria deseada (Pata 1) -----
Xtraj = squeeze(out.X_TRAJ);
Ztraj = squeeze(out.Z_TRAJ);

X_des = Xtraj(1,:);
Z_des = Ztraj(1,:);

%----- Figura -----
figure

plot(X_des,Z_des,'b','LineWidth',2)

hold on

plot(X_real,Z_real,'r--','LineWidth',2)

grid on
axis equal

xlabel('X Position (mm)')
ylabel('Z Position (mm)')

legend('Desired trajectory','Measured trajectory','Location','best')

title('Desired vs Measured Cartesian Foot Trajectory')