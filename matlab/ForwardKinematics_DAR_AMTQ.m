
function [x,y,z,q_real] = ForwardKinematics_DAR_AMTQ(q1,q2,q3,S)

%==========================================================================
% CINEMÁTICA DIRECTA DAR-AMTQ
%
% Validación de InverseKinematics_DAR_AMTQ
%
% Entradas:
% q1,q2,q3 : ángulos Simscape [rad]
% S        : simetría de coxas
%
% Salidas:
% x,y,z    : posición del pie [mm]
% q_real   : ángulos corregidos para depuración
%==========================================================================


%% Parámetros CAD

L1 = 54.32;
L2 = 80.00;
L3 = 161.64;


%% Recuperar ángulos geométricos

q1_real = q1 + S*0.4636;

q2_real = q2 + 0.1273;

q3_real = q3 + 1.8338;


q_real = [
    q1_real;
    q2_real;
    q3_real
];


%% Cinemática directa

theta3 = q2_real - q3_real;

R = L1 + ...
    L2*cos(q2_real) + ...
    L3*cos(theta3);

z = L2*sin(q2_real) + ...
    L3*sin(theta3);

%% Rotación de la coxa

x = R*cos(q1_real);

y = R*sin(q1_real);

disp("q reales")
disp([q1_real q2_real q3_real])

disp("R")
disp(R)

disp("xyz")
disp([x y z])

end