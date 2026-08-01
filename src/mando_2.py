import pygame
import socket
import struct
import time

# --- CONFIGURACIÓN DE RED ---
UDP_IP = "127.0.0.1" 
UDP_PORT = 25000     

print(f"Iniciando transmisión UDP a {UDP_IP}:{UDP_PORT}")       
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("¡ERROR: No se detectó ningún control de Xbox conectado!")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Control detectado: {joystick.get_name()}")

# Memorias para detectar si hubo un cambio real en el control
last_vel_x = -999.0
last_vel_y = -999.0
last_giro_z = -999.0
last_btn_a = -1.0 # Agregamos memoria para el botón A
last_btn_b = -1.0 # Agregamos memoria para el botón B

try:
    while True:
        pygame.event.pump() 

        # Leer palancas e invertir Y
        vel_x = -joystick.get_axis(1) 
        vel_y = joystick.get_axis(0)
        giro_z = joystick.get_axis(3)

        # Leer Botones (En Xbox: 0 es A, 1 es B)
        # Los convertimos a float (decimal) para que todo el paquete sea del mismo tipo
        btn_a = float(joystick.get_button(0)) 
        btn_b = float(joystick.get_button(1))

        # Zona muerta
        if abs(vel_x) < 0.1: vel_x = 0.0
        if abs(vel_y) < 0.1: vel_y = 0.0
        if abs(giro_z) < 0.1: giro_z = 0.0

        # REDONDEO
        vel_x = round(vel_x, 2)
        vel_y = round(vel_y, 2)
        giro_z = round(giro_z, 2)

        # EL FILTRO MÁGICO: Incluimos los botones en la revisión de cambios
        if (vel_x != last_vel_x) or (vel_y != last_vel_y) or (giro_z != last_giro_z) or (btn_a != last_btn_a) or (btn_b != last_btn_b):
            
            # Empaquetamos 5 doubles: '>ddddd'
            mensaje = struct.pack('>ddddd', vel_x, vel_y, giro_z, btn_a, btn_b)
            sock.sendto(mensaje, (UDP_IP, UDP_PORT))
            
            print(f"ORDEN -> X: {vel_x:.2f} | Y: {vel_y:.2f} | Z: {giro_z:.2f} | A: {int(btn_a)} | B: {int(btn_b)}")
            
            # Actualizamos la memoria
            last_vel_x = vel_x
            last_vel_y = vel_y
            last_giro_z = giro_z
            last_btn_a = btn_a
            last_btn_b = btn_b

        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nTransmisión detenida por el usuario.")
    pygame.quit()