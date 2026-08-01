function Link = DAR_Link(P0,P1,axis)
%==========================================================================
% DAR_LINK
%
% Crea un eslabón para el algoritmo DAR.
%
% Entradas:
%   P0   Punto inicial [3x1]
%   P1   Punto final   [3x1]
%   axis 'x','y','z'
%
% Salida:
%   Link estructura del eslabón
%==========================================================================

Link.P0 = P0(:);
Link.P1 = P1(:);

Link.axis = lower(axis);

Link.vector = Link.P1-Link.P0;

Link.length = norm(Link.vector);

end