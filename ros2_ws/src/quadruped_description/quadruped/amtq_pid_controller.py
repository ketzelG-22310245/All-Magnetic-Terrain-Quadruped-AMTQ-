#!/usr/init/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class AMTQPIDController(Node):
    def __init__(self):
        super().__init__('amtq_pid_controller')

        # Ganancias exactas del Reporte de Diseño PID - Perfil Rígido (Sección 4)
        # Kp = 15.0 Nm/rad, Ki = 25.0 Nm/(rad*s), Kd = 0.30 Nm*s/rad
        self.kp = np.full(12, 15.0)
        self.ki = np.full(12, 25.0)
        self.kd = np.full(12, 0.30)

        self.max_torque = 10.0      # Límite nominal del actuador en robot.urdf (10 Nm)
        self.max_integral = 0.40    # Permite hasta 10.0 Nm de corrección integral (25.0 * 0.40)

        self.joint_names = [
            'front_left_coxa_joint', 'front_left_femur_joint', 'front_left_tibia_joint',
            'front_right_coxa_joint', 'front_right_femur_joint', 'front_right_tibia_joint',
            'back_left_coxa_joint', 'back_left_femur_joint', 'back_left_tibia_joint',
            'back_right_coxa_joint', 'back_right_femur_joint', 'back_right_tibia_joint'
        ]

        # Masas identificadas en el URDF para compensación analítica exacta de gravedad
        # [coxa_mass, femur_mass, tibia_mass] aprox por pata
        self.mass_femur = 0.0596   # kg
        self.mass_tibia = 0.2215   # kg
        self.L_femur = 0.08        # m (80 mm)
        self.L_tibia = 0.1616      # m (161.64 mm)
        self.g = 9.81              # m/s^2

        # Postura HOME exacta de DAR IK para (x=100, y=0, z=-135) con base 100% plana
        # Postura HOME exacta de DAR IK para (x=100, y=0, z=-135) con base 100% plana
        self.home_posture = np.array([
            0.0, 0.1734, 0.1734,  # front_left (q2 == q3)
            0.0, 0.1734, 0.1734,  # front_right (q2 == q3)
            0.0, 0.1734, 0.1734,  # back_left (q2 == q3)
            0.0, 0.0112, 0.1734   # back_right (q2 = q3 - 0.1622)
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
        self.total_warmup_steps = 150

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

        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('Controlador PID Analítico con Compensación de Gravedad iniciado.')

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
                    self.dq_filtered[i] = 0.6 * self.dq_filtered[i] + 0.4 * msg.velocity[idx]
        if not self.states_received:
            self.q_initial = self.q_meas.copy()
            self.states_received = True

    def compute_gravity_compensation(self, q):
        """Calcula analíticamente el torque de gravedad Tau_g(q) para las 12 articulaciones"""
        tau_g = np.zeros(12)
        for leg in range(4):
            idx_femur = leg * 3 + 1
            idx_tibia = leg * 3 + 2
            
            q2 = q[idx_femur]
            q3 = q[idx_tibia]
            
            # Modelo dinámico de par gravitacional para fémur y tibia
            # Tau_femur = (m_femur * g * L_femur_com * cos(q2)) + (m_tibia * g * (L_femur * cos(q2) + L_tibia_com * cos(q2 - q3)))
            tau_femur_g = (self.mass_femur * self.g * (self.L_femur / 2.0) * np.cos(q2)) + \
                          (self.mass_tibia * self.g * (self.L_femur * np.cos(q2) + (self.L_tibia / 2.0) * np.cos(q2 - q3)))
            
            # Tau_tibia = m_tibia * g * L_tibia_com * cos(q2 - q3)
            tau_tibia_g = self.mass_tibia * self.g * (self.L_tibia / 2.0) * np.cos(q2 - q3)
            
            tau_g[idx_femur] = tau_femur_g
            tau_g[idx_tibia] = tau_tibia_g
            # Coxa soporta principalmente carga lateral (se deja en 0 o fricción mínima)
            tau_g[leg * 3] = 0.0
            
        return tau_g

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

        # 1. Ley de control PID estándar
        tau_pid = (self.kp * error) + (self.ki * self.integral_err) + (self.kd * d_error)

        # 2. Término de prealimentación por modelo analítico de gravedad (Torque Feedforward)
        tau_grav = self.compute_gravity_compensation(self.q_meas)

        # Torque total = PID + Compensación Gravitacional Analítica
        tau = tau_pid + tau_grav
        tau = np.clip(tau, -self.max_torque, self.max_torque)

        msg = Float64MultiArray()
        msg.data = tau.tolist()
        self.pub_effort.publish(msg)

    def send_zero_torque(self):
        if rclpy.ok():
            msg = Float64MultiArray()
            msg.data = [0.0] * 12
            self.pub_effort.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AMTQPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_zero_torque()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()