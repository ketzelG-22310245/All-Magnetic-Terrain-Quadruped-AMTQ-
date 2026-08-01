%% ============================================================
% Joint Angular Velocities
% Adapted DAR Method - AMTQ
% ============================================================

t = out.tout;

Q1 = squeeze(out.Q1);
Q2 = squeeze(out.Q2);
Q3 = squeeze(out.Q3);

% Pata delantera derecha
q1 = Q1(1,:);
q2 = Q2(1,:);
q3 = Q3(1,:);

% Incremento temporal
dt = mean(diff(t));

% Velocidades articulares
dq1 = gradient(q1,dt);
dq2 = gradient(q2,dt);
dq3 = gradient(q3,dt);

figure('Color','w','Position',[200 100 900 700]);

subplot(3,1,1)

plot(t,dq1,'LineWidth',2)

grid on
box on

ylabel('$\dot{q}_1$ (rad/s)','Interpreter','latex')

title('Joint Angular Velocities using the Adapted DAR Method')

subplot(3,1,2)

plot(t,dq2,'LineWidth',2)

grid on
box on

ylabel('$\dot{q}_2$ (rad/s)','Interpreter','latex')

subplot(3,1,3)

plot(t,dq3,'LineWidth',2)

grid on
box on

ylabel('$\dot{q}_3$ (rad/s)','Interpreter','latex')

xlabel('Time (s)')

sgtitle({'Angular Velocity Profiles',...
         'Adapted Directed Angular Restitution (DAR)'},...
         'FontWeight','bold','FontSize',14)