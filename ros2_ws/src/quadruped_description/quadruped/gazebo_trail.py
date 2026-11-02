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
        self.sub_def = self.create_subscription(
            TFMessage, '/world/default/dynamic_pose/info', self.pose_cb, 10
        )
        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.start_t = None
        self.last_marker_x = None
        self.last_marker_y = None
        self.marker_id = 0
        self.counter = 0

        self.csv_path = 'trajectory_log.csv'
        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time_s', 'x_m', 'y_m', 'z_m', 'dx_mm', 'dy_mm', 'dist_mm', 'yaw_deg'])
        self.csv_file.flush()
        self.get_logger().info('Trazador iniciado. Esperando datos de /world/default/dynamic_pose/info...')

    def get_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def pose_cb(self, msg):
        if not msg.transforms:
            return
        t = msg.transforms[0]
        x = t.transform.translation.x
        y = t.transform.translation.y
        z = t.transform.translation.z
        yaw = self.get_yaw(t.transform.rotation)
        now = time.time()

        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.start_yaw = yaw
            self.start_t = now
            self.last_marker_x = x
            self.last_marker_y = y
            self.get_logger().info('¡Conectado! Dibujando estela verde en Gazebo Sim...')

        self.counter += 1
        if self.counter % 5 != 0:
            return

        elapsed = now - self.start_t
        dx = (x - self.start_x) * 1000.0
        dy = (y - self.start_y) * 1000.0
        raw_dyaw = yaw - self.start_yaw
        dyaw = math.degrees(math.atan2(math.sin(raw_dyaw), math.cos(raw_dyaw)))
        dist = math.hypot(dx, dy)

        self.csv_writer.writerow([
            f'{elapsed:.3f}', f'{x:.5f}', f'{y:.5f}', f'{z:.5f}',
            f'{dx:.2f}', f'{dy:.2f}', f'{dist:.2f}', f'{dyaw:.3f}'
        ])
        self.csv_file.flush()

        # Spawnear punto verde físico sin colisión cada 8 mm de recorrido
        step_dist = math.hypot(x - self.last_marker_x, y - self.last_marker_y)
        if step_dist >= 0.008 or self.marker_id == 0:
            self.last_marker_x = x
            self.last_marker_y = y
            self.marker_id += 1

            sdf_dot = (
                f"<sdf version='1.8'><model name='trail_dot_{self.marker_id}'>"
                f"<static>true</static><pose>{x:.4f} {y:.4f} 0.002 0 0 0</pose>"
                f"<link name='link'><visual name='vis'><geometry><box>"
                f"<size>0.014 0.014 0.003</size></box></geometry>"
                f"<material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse>"
                f"<emissive>0 1 0 1</emissive></material>"
                f"</visual></link></model></sdf>"
            )
            req = f'sdf: "{sdf_dot}"'
            for world_name in ['empty', 'default']:
                subprocess.Popen(
                    ['gz', 'service', '-s', f'/world/{world_name}/create',
                     '--reqtype', 'gz.msgs.EntityFactory',
                     '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '200', '--req', req],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )

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
