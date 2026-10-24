#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import csv
import time

class StepIdentificationNode(Node):
    def __init__(self):
        super().__init__('step_identification_node')
        
        # Parámetros de la prueba de escalón
        self.target_joint = 'front_left_femur_joint'
        self.step_torque = 0.5       # Torque del escalón en Nm (ajústalo si es muy alto o bajo)
        self.pre_step_time = 1.0     # Tiempo en 0 Nm antes del escalón (segundos)
        self.step_duration = 4.0     # Duración del escalón activo (segundos)
        self.csv_filename = 'femur_step_response.csv'

        # Publicador al controlador de esfuerzo y suscriptor a estados
        self.pub_effort = self.create_publisher(
            Float64MultiArray, 
            '/leg_effort_controller/commands', 
            10
        )
        self.sub_states = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )

        self.data_log = []
        self.start_time = None
        self.current_cmd = 0.0
        self.finished = False

        # Bucle de control a 100 Hz (0.01 s)
        self.timer = self.create_timer(0.01, self.control_loop)
        self.get_logger().info(f'Iniciando prueba de escalón para {self.target_joint}...')

    def joint_state_callback(self, msg):
        if self.finished:
            return
            
        if self.target_joint in msg.name:
            idx = msg.name.index(self.target_joint)
            now = time.time()
            if self.start_time is None:
                return
                
            elapsed = now - self.start_time
            pos = msg.position[idx] if len(msg.position) > idx else 0.0
            vel = msg.velocity[idx] if len(msg.velocity) > idx else 0.0
            eff = msg.effort[idx] if len(msg.effort) > idx else 0.0

            self.data_log.append([elapsed, self.current_cmd, pos, vel, eff])

    def control_loop(self):
        if self.start_time is None:
            self.start_time = time.time()

        elapsed = time.time() - self.start_time
        msg = Float64MultiArray()

        if elapsed < self.pre_step_time:
            self.current_cmd = 0.0
        elif elapsed < (self.pre_step_time + self.step_duration):
            self.current_cmd = self.step_torque
        else:
            # Finalizar prueba y apagar motor
            self.current_cmd = 0.0
            msg.data = [0.0]
            self.pub_effort.publish(msg)
            self.save_csv()
            self.finished = True
            self.timer.cancel()
            raise SystemExit

        msg.data = [self.current_cmd]
        self.pub_effort.publish(msg)

    def save_csv(self):
        with open(self.csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['time_s', 'torque_cmd_Nm', 'position_rad', 'velocity_rad_s', 'effort_measured_Nm'])
            writer.writerows(self.data_log)
        self.get_logger().info(f'Prueba completada. {len(self.data_log)} muestras guardadas en {self.csv_filename}')

def main(args=None):
    rclpy.init(args=args)
    node = StepIdentificationNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()