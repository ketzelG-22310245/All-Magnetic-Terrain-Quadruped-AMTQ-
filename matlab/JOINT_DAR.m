%% ============================================================
% Joint Angle Evolution
% Adapted Directed Angular Restitution (DAR) Method
% AMTQ Quadruped Robot
% ============================================================

% Tiempo de simulación
t = out.tout;

% Ángulos articulares registrados
Q1 = squeeze(out.Q1);
Q2 = squeeze(out.Q2);
Q3 = squeeze(out.Q3);

% Seleccionar la pata delantera derecha (Pata 1)
q1 = Q1(1,:);
q2 = Q2(1,:);
q3 = Q3(1,:);

% ============================================================
% Figura
% ============================================================

figure('Color','w','Position',[200 100 900 700]);

%-----------------------------
% q1
%-----------------------------
subplot(3,1,1)

plot(t,q1,'LineWidth',2)

grid on
box on

ylabel('$q_1$ (rad)','Interpreter','latex','FontSize',12)

title('Joint Angle Evolution using the Adapted DAR Method',...
    'FontWeight','bold')

xlim([t(1) t(end)])

%-----------------------------
% q2
%-----------------------------
subplot(3,1,2)

plot(t,q2,'LineWidth',2)

grid on
box on

ylabel('$q_2$ (rad)','Interpreter','latex','FontSize',12)

xlim([t(1) t(end)])

%-----------------------------
% q3
%-----------------------------
subplot(3,1,3)

plot(t,q3,'LineWidth',2)

grid on
box on

ylabel('$q_3$ (rad)','Interpreter','latex','FontSize',12)
xlabel('Time (s)','FontSize',12)

xlim([t(1) t(end)])

%===========================================================
% Título general
%===========================================================
sgtitle({'Joint Angle Evolution of the AMTQ Robot',...
         'Adapted Directed Angular Restitution (DAR) Method'},...
         'FontWeight','bold','FontSize',14)