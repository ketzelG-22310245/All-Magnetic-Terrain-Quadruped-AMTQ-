#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class AMTQPIDController(Node):
    def __init__(self):
        super().__init__('amtq_pid_controller')

        # Ganancias estables para inercia ligera (coxa, femur, tibia) sin chattering discreto
        self.kp = np.array([
            8.0, 12.0, 10.0,   # Front Left
            8.0, 12.0, 10.0,   # Front Right
            8.0, 12.0, 10.0,   # Back Left
            8.0, 12.0, 10.0    # Back Right
        ])
        self.ki = np.array([
            2.0, 6.0, 4.0,
            2.0, 6.0, 4.0,
            2.0, 6.0, 4.0,
            2.0, 6.0, 4.0
        ])
        self.kd = np.array([
            0.05, 0.10, 0.08,
            0.05, 0.10, 0.08,
            0.05, 0.10, 0.08,
            0.05, 0.10, 0.08
        ])

        self.max_torque = 4.0       # Límite suave para evitar latigazos
        self.max_integral = 0.25

        self.joint_names = [
            'front_left_coxa_joint', 'front_left_femur_joint', 'front_left_tibia_joint',
            'front_right_coxa_joint', 'front_right_femur_joint', 'front_right_tibia_joint',
            'back_left_coxa_joint', 'back_left_femur_joint', 'back_left_tibia_joint',
            'back_right_coxa_joint', 'back_right_femur_joint', 'back_right_tibia_joint'
        ]

        # Postura HOME 100% plana sobre el suelo (q2 = q3 mantiene la tibia vertical)
        self.home_posture = np.array([
            0.0,  0.0, 0.0,     # Front Left  (100% plano)
            0.0,  0.0, 0.0,     # Front Right (100% plano)
            0.0,  0.0, 0.0,     # Back Left   (100% plano)
            0.0, -0.1622, 0.0   # Back Right  (compensa el offset de 0.1622 rad del URDF)
        ])

        self.q_ref = self.home_posture.copy()
        self.q_initial = None
        self.q_meas = np.zeros(12)
        self.dq_meas = np.zeros(12)
        self.dq_filtered = np.zeros(12)
        self.integral_err = np.zeros(12)

        self.states_received = False
        self.gait_active = False
        self.warmup_steps = 0
        self.total_warmup_steps = 150  # 1.5 s de rampa suave

        self.sub_ref = self.create_subscription(
            Float64MultiArray,
            '/leg_position_controller/commands',
            self.ref_callback,
            10
        )
        self.sub_states = self.create_subscription(
            JointState,
            '/joint_states',
            self.state_callback,
            10
        )
        self.pub_effort = self.create_publisher(
            Float64MultiArray,
            '/leg_effort_controller/commands',
            10
        )

        # Lazo a 100 Hz sincronizado con Gazebo
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('Controlador PID AMTQ estable iniciado.')

    def ref_callback(self, msg):
        if len(msg.data) == 12:
            self.q_ref = np.array(msg.data, dtype=float)
            self.gait_active = True

    def state_callback(self, msg):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.q_meas[i] = msg.position[idx]
                if len(msg.velocity) > idx:
                    # Filtro pasa-bajas exponencial para eliminar ruido/vibración derivativa
                    self.dq_filtered[i] = 0.6 * self.dq_filtered[i] + 0.4 * msg.velocity[idx]
        if not self.states_received:
            self.q_initial = self.q_meas.copy()
            self.states_received = True

    def control_loop(self):
        if not self.states_received:
            return

        if not self.gait_active and self.warmup_steps < self.total_warmup_steps:
            alpha = float(self.warmup_steps) / float(self.total_warmup_steps)
            target_q = (1.0 - alpha) * self.q_initial + alpha * self.home_posture
            self.warmup_steps += 1
        else:
            target_q = self.q_ref

        error = target_q - self.q_meas

        self.integral_err += error * self.dt
        self.integral_err = np.clip(self.integral_err, -self.max_integral, self.max_integral)

        d_error = -self.dq_filtered

        tau = (self.kp * error) + (self.ki * self.integral_err) + (self.kd * d_error)
        tau = np.clip(tau, -self.max_torque, self.max_torque)

        msg = Float64MultiArray()
        msg.data = tau.tolist()
        self.pub_effort.publish(msg)

    def send_zero_torque(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * 12
        self.pub_effort.publish(msg)
        self.get_logger().info('Torques en 0.0 Nm enviados. Apagando PID limpiamente.')

def main(args=None):
    rclpy.init(args=args)
    node = AMTQPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.send_zero_torque()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()