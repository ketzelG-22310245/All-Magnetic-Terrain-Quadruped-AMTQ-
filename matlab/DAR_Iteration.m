function [dq1,dq2,dq3] = DAR_Iteration(J1,J2,J3,Foot,Target)
%==========================================================================
% DAR_ITERATION
%
% Adaptación del método Directed Angular Restitution (DAR)
% para el AMTQ.
%
% Entradas:
%
%   J1      Articulación Coxa   [3x1]
%   J2      Articulación Fémur  [3x1]
%   J3      Articulación Tibia  [3x1]
%   Foot    Pie                 [3x1]
%   Target  Objetivo cartesiano [3x1]
%
% Salidas:
%
%   dq1,dq2,dq3  Incrementos articulares [rad]
%
%==========================================================================

%%----------------------------------------------------------
%% Vectores del robot
%%----------------------------------------------------------

rAB = J2 - J1;      % Coxa
rBC = J3 - J2;      % Fémur
rCD = Foot - J3;    % Tibia

%%----------------------------------------------------------
%% Vectores hacia el objetivo
%%----------------------------------------------------------

rAT = Target - J1;

rBT = Target - J2;

rCT = Target - J3;

%%----------------------------------------------------------
%% Restitución angular
%%----------------------------------------------------------

dq1 = DAR_Angle( ...
    rAB,...
    rAT,...
    'z',...
    3);

dq2 = DAR_Angle( ...
    rBC,...
    rBT,...
    'y',...
    3);

dq3 = DAR_Angle( ...
    rCD,...
    rCT,...
    'y',...
    3);

end