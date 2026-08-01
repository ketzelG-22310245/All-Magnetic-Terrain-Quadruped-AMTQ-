%% Trayectoria Cartesiana del Pie

XYZ = squeeze(out.XYZ_REAL);

X = XYZ(1,:);
Y = XYZ(2,:);
Z = XYZ(3,:);

figure
plot(X,Z,'LineWidth',2)

grid on
axis equal

xlabel('X Position (m)')
ylabel('Z Position (m)')
title('Cartesian Foot Trajectory')

set(gca,'FontSize',12)