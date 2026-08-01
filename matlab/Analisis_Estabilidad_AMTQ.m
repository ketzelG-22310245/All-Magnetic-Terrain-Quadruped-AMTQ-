% =========================================================================
% TÍTULO: Análisis de Estabilidad en el Espacio de Estados - AMTQ v2.0
% DESCRIPCIÓN: Cálculo de polos, ceros y respuesta transitoria de los 
%              actuadores DS3235 (Coxa) y DS3225 (Fémur/Tibia).
% =========================================================================
clear; clc; close all;

%% 1. DEFINICIÓN DE PARÁMETROS FÍSICOS (Unidades SI)
% Parámetros extraídos de Datasheets y estimaciones literarias
La = 0.002;          % Inductancia de armadura (Henrios)
Jm = 2.5e-5;         % Inercia del rotor interno (kg*m^2)
Bm = 1.5e-3;         % Fricción viscosa estimada (N*m*s/rad)

% --- Motor 1: Coxa (DS3235 - 35 Kg) ---
Ra1 = 3.21;          % Resistencia de armadura (Ohms)
Kt1 = 1.49;          % Constante de torque (Nm/A)
Ke1 = 1.49;          % Constante de fuerza contraelectromotriz (V*s/rad)

% --- Motor 2: Fémur/Tibia (DS3225 - 25 Kg) ---
Ra2 = 2.95;          % Resistencia de armadura (Ohms)
Kt2 = 1.04;          % Constante de torque (Nm/A)
Ke2 = 1.04;          % Constante de fuerza contraelectromotriz (V*s/rad)

%% 2. CONSTRUCCIÓN DE LAS MATRICES DE ESTADO (A, B, C, D)
% Matriz A: Matriz de Dinámica del Sistema
% Variables de estado: x1 = Posición, x2 = Velocidad, x3 = Corriente
A_coxa = [ 0,       1,          0; 
           0, -(Bm/Jm),  (Kt1/Jm); 
           0, -(Ke1/La), -(Ra1/La)];
       
A_femur = [ 0,       1,          0; 
            0, -(Bm/Jm),  (Kt2/Jm); 
            0, -(Ke2/La), -(Ra2/La)];

% Matriz B: Matriz de Entrada (Control = Voltaje Va)
B_sys = [0; 0; 1/La];

% Matriz C: Matriz de Salida (Sensor = Posición Angular)
C_sys = [1, 0, 0];

% Matriz D: Transmisión directa (Nula)
D_sys = 0;

%% 3. CREACIÓN DE LOS MODELOS EN ESPACIO DE ESTADOS (SS)
sys_coxa = ss(A_coxa, B_sys, C_sys, D_sys);
sys_femur = ss(A_femur, B_sys, C_sys, D_sys);

% Cálculo analítico de los Polos (Eigenvalores de la Matriz A)
polos_coxa = pole(sys_coxa);
polos_femur = pole(sys_femur);

disp('=== POLOS DEL SISTEMA DE LA COXA (DS3235) ===');
disp(polos_coxa);
disp('=== POLOS DEL SISTEMA DEL FÉMUR (DS3225) ===');
disp(polos_femur);

%% 4. GRÁFICAS PARA EL DOCUMENTO DE INVESTIGACIÓN

% Figura 1: Mapa de Polos y Ceros (Estabilidad)
figure('Name', 'Análisis de Estabilidad (Polos)', 'Color', 'w');
pzmap(sys_coxa, 'b', sys_femur, 'r');
title('Mapa de Polos y Ceros - Dinámica de Actuadores');
legend('Coxa (DS3235)', 'Fémur/Tibia (DS3225)', 'Location', 'best');
grid on;

% Figura 2: Respuesta al Escalón (Comportamiento Transitorio)
% Simula cómo reacciona el motor si le aplicamos 1V de golpe.
figure('Name', 'Respuesta al Escalón', 'Color', 'w');
step(sys_coxa, 'b', sys_femur, 'r');
title('Respuesta Dinámica a un Escalón de Voltaje (Lazo Abierto)');
xlabel('Tiempo (segundos)');
ylabel('Posición Angular (radianes)');
legend('Coxa (DS3235)', 'Fémur/Tibia (DS3225)', 'Location', 'best');
grid on;