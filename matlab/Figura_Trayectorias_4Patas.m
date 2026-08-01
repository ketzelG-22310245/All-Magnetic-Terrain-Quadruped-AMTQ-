clc
clear
close all

%%==========================================================
% TRAYECTORIAS DE LAS 4 PATAS - AMTQ
% SOLO VISTA 3D
%==========================================================

L = 80;
H = 40;
T = 1;

desfase = [0 T/2 T T/2];

dt = 0.01;
t = 0:dt:T;

N = length(t);


%%==========================================================
% GEOMETRÍA ROBOT
%%==========================================================

BodyLength = 180;
BodyWidth  = 120;


Hip = [

 BodyLength/2   BodyWidth/2
 BodyLength/2  -BodyWidth/2
-BodyLength/2   BodyWidth/2
-BodyLength/2  -BodyWidth/2

];


%%==========================================================
% ALMACENAMIENTO
%%==========================================================

X=zeros(N,4);
Y=zeros(N,4);
Z=zeros(N,4);



%%==========================================================
% GENERADOR DE TRAYECTORIAS
%%==========================================================

for k=1:N

    [x,y,z]=generador_paso_eliptico_gateo(...
        t(k),L,H,T,desfase);


    X(k,:)=x';
    Y(k,:)=y';
    Z(k,:)=z';

end



%%==========================================================
% TRANSFORMACIÓN AL MARCO DEL ROBOT
%%==========================================================

Xr=zeros(size(X));
Yr=zeros(size(Y));


theta=[

 pi/4
-pi/4
 3*pi/4
-3*pi/4

];


%%==========================================================
% ESCALA VISUAL DE TRAYECTORIA
% SOLO PARA GRAFICA
%%==========================================================

scale_xy = 1.35;     % aumenta ancho de paso


for i=1:4

    c=cos(theta(i));
    s=sin(theta(i));


    % Aumentar trayectoria respecto a la coxa

    Xlocal = scale_xy * X(:,i);

    Ylocal = scale_xy * Y(:,i);



    % Rotación según orientación de pata

    Xr(:,i)=c*Xlocal-s*Ylocal+Hip(i,1);

    Yr(:,i)=s*Xlocal+c*Ylocal+Hip(i,2);


end


%%==========================================================
% AJUSTE DE Z PARA VISUALIZACIÓN
% EL ARCO DE ELEVACIÓN QUEDA ARRIBA
% RANGO 50 - 200 mm
%%==========================================================


Zr=-Z;


% mover mínimo a cero

Zr=Zr-min(Zr(:));



Zmin=50;
Zmax=250;



for i=1:4


    zi=Zr(:,i);


    % normalizar 0-1

    if max(zi)>0

        zi=zi/max(zi);

    end



    % escalar entre 50 y 200

    Zr(:,i)=Zmin + zi*(Zmax-Zmin);


end



%%==========================================================
% GRAFICA 3D
%%==========================================================


figure('Color','w',...
'Position',[100 80 950 750])


hold on
grid on



C=lines(4);

h=gobjects(4,1);



for i=1:4


    % trayectoria

    h(i)=plot3(...
        Xr(:,i),...
        Yr(:,i),...
        Zr(:,i),...
        'Color',C(i,:),...
        'LineWidth',3);



    % inicio

    plot3(...
        Xr(1,i),...
        Yr(1,i),...
        Zr(1,i),...
        'o',...
        'Color',C(i,:),...
        'MarkerFaceColor',C(i,:),...
        'MarkerSize',8);



    % final

    plot3(...
        Xr(end,i),...
        Yr(end,i),...
        Zr(end,i),...
        's',...
        'Color',C(i,:),...
        'MarkerFaceColor',C(i,:),...
        'MarkerSize',7);

end



%%==========================================================
% CONFIGURACIÓN DE EJES
%%==========================================================


xlabel('X [mm]')
ylabel('Y [mm]')
zlabel('Z [mm]')


title('AMTQ Foot Trajectories 3D')


view(45,25)



axis equal

grid on



% Alcance aproximado del robot

xlim([-350 350])

ylim([-350 350])

zlim([0 250])



daspect([1 1 1])



legend(h,...
'Leg 1',...
'Leg 2',...
'Leg 3',...
'Leg 4',...
'Location','best')



%%==========================================================
% EXPORTAR
%%==========================================================

exportgraphics(...
gcf,...
'AMTQ_Foot_Trajectories_3D.png',...
'Resolution',600);