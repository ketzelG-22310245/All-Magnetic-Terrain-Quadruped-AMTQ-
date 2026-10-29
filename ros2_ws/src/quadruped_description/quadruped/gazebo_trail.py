#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import subprocess
import math
import csv
import time

class GazeboTrail(Node):
    def __init__(self):
        super().__init__('gazebo_trail')
        # Suscribirse tanto a 'default' (nombre real en tu Entity Tree) como a 'empty'
        self.sub_def = self.create_subscription(
            TFMessage, '/world/default/dynamic_pose/info', self.pose_cb, 10
        )
        self.sub_emp = self.create_subscription(
            TFMessage, '/world/empty/dynamic_pose/info', self.pose_cb, 10
        )
        self.points = []
        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.start_t = None
        self.last_marker_x = None
        self.last_marker_y = None
        self.marker_id = 10
        self.counter = 0

        self.csv_path = 'trajectory_log.csv'
        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time_s', 'x_m', 'y_m', 'z_m', 'dx_mm', 'dy_mm', 'dist_mm', 'yaw_deg'])
        self.csv_file.flush()

        self.get_logger().info(f'Trazador listo. Guardando en {self.csv_path}. Esperando puente de Gazebo...')

    def get_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def pose_cb(self, msg):
        if not msg.transforms:
            return

        # Buscar específicamente el chasis del robot (quadruped o base_link), o tomar el primero
        target_tf = msg.transforms[0]
        for tf in msg.transforms:
            name = tf.child_frame_id.lower()
            if 'quadruped' in name or 'base_link' in name:
                target_tf = tf
                break

        x = target_tf.transform.translation.x
        y = target_tf.transform.translation.y
        z = target_tf.transform.translation.z
        yaw = self.get_yaw(target_tf.transform.rotation)
        now = time.time()

        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.start_yaw = yaw
            self.start_t = now
            self.last_marker_x = x
            self.last_marker_y = y
            self.get_logger().info(f'¡Datos recibidos! Frame detectado: "{target_tf.child_frame_id}"')

        self.counter += 1
        if self.counter % 5 != 0:
            return

        elapsed = now - self.start_t
        dx = (x - self.start_x) * 1000.0
        dy = (y - self.start_y) * 1000.0
        dyaw = math.degrees(yaw - self.start_yaw)
        dist = math.hypot(dx, dy)

        # 1. Guardar en CSV siempre
        self.csv_writer.writerow([
            f'{elapsed:.3f}', f'{x:.5f}', f'{y:.5f}', f'{z:.5f}',
            f'{dx:.2f}', f'{dy:.2f}', f'{dist:.2f}', f'{dyaw:.3f}'
        ])
        self.csv_file.flush()

        # 2. Dibujar marcador en Gazebo (Esfera verde + Línea para asegurar visibilidad en OpenGL)
        step_dist = math.hypot(x - self.last_marker_x, y - self.last_marker_y)
        if step_dist >= 0.005 or self.counter == 5:  # Cada 5 mm de avance pone un punto
            self.last_marker_x = x
            self.last_marker_y = y
            self.marker_id += 1

            # Esfera verde de 1.2 cm de diámetro sobre el suelo
            req_sphere = (
                f'action: ADD_MODIFY, ns: "amtq_dots", id: {self.marker_id}, type: SPHERE, '
                f'lifetime {{sec: 0}}, '
                f'pose {{position {{x: {x:.4f}, y: {y:.4f}, z: 0.006}}}}, '
                f'scale {{x: 0.012, y: 0.012, z: 0.012}}, '
                f'material {{ambient {{r: 0.0, g: 1.0, b: 0.0, a: 1.0}}, '
                f'diffuse {{r: 0.0, g: 1.0, b: 0.0, a: 1.0}}}}'
            )
            sdf_dot = (
                f"<sdf version='1.8'><model name='dot_{self.marker_id}'>"
                f"<static>true</static><pose>{x:.4f} {y:.4f} 0.002 0 0 0</pose>"
                f"<link name='l'><visual name='v'><geometry><cylinder>"
                f"<radius>0.006</radius><length>0.002</length></cylinder></geometry>"
                f"<material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse></material>"
                f"</visual></link></model></sdf>"
            )
            req_create = f'sdf: "{sdf_dot}"'
            subprocess.Popen(
                ['gz', 'service', '-s', '/world/default/create',
                 '--reqtype', 'gz.msgs.EntityFactory',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '100', '--req', req_create],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )

        # 3. Imprimir en terminal cada 20 muestras
        if self.counter % 20 == 0:
            self.get_logger().info(
                f't: {elapsed:5.1f}s | dX: {dx:+6.1f} mm | dY: {dy:+6.1f} mm | '
                f'Dist: {dist:5.1f} mm | Yaw: {dyaw:+5.2f} deg'
            )

    def close_csv(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    node = GazeboTrail()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_csv()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
