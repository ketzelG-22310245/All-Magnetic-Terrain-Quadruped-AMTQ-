%% ============================================================
% Joint Angular Accelerations
% Adapted DAR Method
% AMTQ Quadruped
% ============================================================

t = out.tout;

dt = mean(diff(t));

%-------------------------------------------------------------
% Aceleraciones
%-------------------------------------------------------------
ddq1 = gradient(dq1,dt);
ddq2 = gradient(dq2,dt);
ddq3 = gradient(dq3,dt);

%-------------------------------------------------------------
% Figura
%-------------------------------------------------------------
figure('Color','w',...
       'Position',[200 100 900 700]);

subplot(3,1,1)

plot(t,ddq1,'LineWidth',2)

grid on
box on

ylabel('$\ddot{q}_1$ (rad/s$^2$)',...
        'Interpreter','latex')

title('Joint Angular Accelerations')

subplot(3,1,2)

plot(t,ddq2,'LineWidth',2)

grid on
box on

ylabel('$\ddot{q}_2$ (rad/s$^2$)',...
        'Interpreter','latex')

subplot(3,1,3)

plot(t,ddq3,'LineWidth',2)

grid on
box on

ylabel('$\ddot{q}_3$ (rad/s$^2$)',...
        'Interpreter','latex')

xlabel('Time (s)')

sgtitle({'Angular Acceleration Profiles',...
         'Adapted Directed Angular Restitution (DAR)'},...
         'FontWeight','bold','FontSize',14)