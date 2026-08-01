function [x, y, z] = generador_paso_eliptico_gateo(t, L, H, T, desfase)
    % =====================================================================
    % GENERADOR DE MARCHA ELÍPTICA - GATEO ELECTROMAGNÉTICO (1 PATA A LA VEZ)
    % Adaptación Fiel al Modelo de Trayectoria Elíptica de Lizarraga et al. (2024)
    % Calibración respetada: Postura en X (0.707), Inversión de Coxas y Fémures
    % =====================================================================
    
    % 0. Prealocación de memoria obligatoria para las 4 patas
    x = zeros(4, 1);
    y = zeros(4, 1);
    z = zeros(4, 1);
    
    % 1. Posición de reposo (Home) - Postura en X del AMTQ V2.0 (mm)
    x_home = 120; 
    y_home = 0;
    z_home = -150; 
    
    % 2. Control de dirección para COXA y FÉMUR (Signos de calibración Simscape)
    signo_y = [1; -1; -1; 1]; 
    signo_z = -1; 
    
    % Redividimos el periodo T: 25% vuelo (despege magnético), 75% apoyo (anclaje)
    T_swing = T / 4;       % Tiempo de vuelo (25%)
    T_stance = 3 * T / 4;  % Tiempo de apoyo (75%)
    
    % 3. Procesamos cada pata con la parametrización elíptica
    for i = 1:4
        % Reloj local con tu secuencia sincronizada
        t_efectivo = t + desfase(i);
        t_ciclo = mod(t_efectivo, T); 
        
        if t_ciclo < T_swing
            % --- FASE DE VUELO ELÍPTICA (Swing Phase) - 25% del tiempo ---
            % En Lizarraga, la fase de vuelo sigue una curva elíptica suave:
            % Avance horizontal armónico (coseno) y elevación vertical (seno)
            phi = (pi * t_ciclo) / T_swing; % Varía de 0 a pi radianes
            
            % Ecuación paramétrica elíptica para el avance (de -L/2 a +L/2)
            paso = (L / 2) * (1 - cos(phi)) - (L / 2);
            
            % Ecuación paramétrica elíptica para la altura (semieje menor H)
            z_offset = H * sin(phi);
        else
            % --- FASE DE APOYO (Stance Phase) - 75% del tiempo ---
            % Desplazamiento lineal de retroceso empujando la superficie metálica
            t_stance_local = t_ciclo - T_swing;
            progreso_stance = t_stance_local / T_stance;
            
            % Retroceso lineal desde +L/2 hasta -L/2
            paso = (L / 2) - (L * progreso_stance);
            z_offset = 0; % Contacto magnético perfecto con el suelo
        end
        
        % 4. DISTRIBUCIÓN DEL PASO EN LA POSTURA EN X (Proyección a 45° = 0.707)
        x(i) = x_home + (paso * 0.707);                 % Avance radial (Fémur/Tibia)
        y(i) = y_home + (paso * 0.707 * signo_y(i));    % Avance tangencial (Coxa)
        z(i) = z_home + (z_offset * signo_z);           % Elevación calibrada para Simscape
    end
end