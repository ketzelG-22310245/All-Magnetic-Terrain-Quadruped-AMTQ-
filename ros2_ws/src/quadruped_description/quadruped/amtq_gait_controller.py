#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class AMTQGaitController(Node):
    """
    ================================================================
    AMTQ V2.0
    DAR Inverse Kinematics + Elliptical Crawling Gait
    ================================================================

    Flujo:

        Tiempo
          |
          v
    Gait Generator
          |
          v
       X, Y, Z
          |
          v
        DAR IK
          |
          v
       q1, q2, q3
          |
          v
    Joint Limits
          |
          v
    12 Joint Commands
          |
          v
    ROS 2 Controller
    """

    # ================================================================
    # INITIALIZATION
    # ================================================================

    def __init__(self):

        super().__init__('amtq_gait_controller')

        # ============================================================
        # JOINT ORDER
        # ============================================================

        # Orden interno:
        #
        # 0 = FL
        # 1 = FR
        # 2 = BL
        # 3 = BR
        #

        self.leg_names = [
            'front_left',
            'front_right',
            'back_left',
            'back_right'
        ]

        # ============================================================
        # JOINT NAMES
        # ============================================================

        self.joint_names = [

            # FRONT LEFT
            'front_left_coxa_joint',
            'front_left_femur_joint',
            'front_left_tibia_joint',

            # FRONT RIGHT
            'front_right_coxa_joint',
            'front_right_femur_joint',
            'front_right_tibia_joint',

            # BACK LEFT
            'back_left_coxa_joint',
            'back_left_femur_joint',
            'back_left_tibia_joint',

            # BACK RIGHT
            'back_right_coxa_joint',
            'back_right_femur_joint',
            'back_right_tibia_joint'
        ]

        # ============================================================
        # PARAMETERS
        # ============================================================

        self.declare_parameter(
            'control_frequency',
            50.0
        )

        self.declare_parameter(
            'step_length',
            80.0
        )

        self.declare_parameter(
            'step_height',
            40.0
        )

        self.declare_parameter(
            'gait_period',
            1.0
        )

        self.declare_parameter(
            'dar_iterations',
            10
        )

        self.declare_parameter(
            'dar_lambda',
            0.005
        )

        self.declare_parameter(
            'home_x',
            120.0
        )

        self.declare_parameter(
            'home_y',
            0.0
        )

        self.declare_parameter(
            'home_z',
            -150.0
        )

        self.declare_parameter(
            'home_duration',
            5.0
        )

        self.declare_parameter(
            'start_gait',
            False
        )
        
        self.declare_parameter(
            'home_test_coxa_zero',
            True
        )

        self.declare_parameter(
            'print_debug',
            True
        )

        self.declare_parameter(
            'publish_joint_states',
            False
        )

        # ============================================================
        # READ PARAMETERS
        # ============================================================

        self.control_frequency = float(
            self.get_parameter(
                'control_frequency'
            ).value
        )

        self.L = float(
            self.get_parameter(
                'step_length'
            ).value
        )

        self.H = float(
            self.get_parameter(
                'step_height'
            ).value
        )

        self.T = float(
            self.get_parameter(
                'gait_period'
            ).value
        )

        self.N_DAR = int(
            self.get_parameter(
                'dar_iterations'
            ).value
        )

        self.lambda_val = float(
            self.get_parameter(
                'dar_lambda'
            ).value
        )

        self.x_home = float(
            self.get_parameter(
                'home_x'
            ).value
        )

        self.y_home = float(
            self.get_parameter(
                'home_y'
            ).value
        )

        self.z_home = float(
            self.get_parameter(
                'home_z'
            ).value
        )

        self.home_duration = float(
            self.get_parameter(
                'home_duration'
            ).value
        )

        self.gait_enabled = bool(
            self.get_parameter(
                'start_gait'
            ).value
        )
        
        self.home_test_coxa_zero = bool(
            self.get_parameter(
                'home_test_coxa_zero'
            ).value
        )

        self.debug = bool(
            self.get_parameter(
                'print_debug'
            ).value
        )

        self.publish_joint_states_enabled = bool(
            self.get_parameter(
                'publish_joint_states'
            ).value
        )

        # ============================================================
        # ROS PUBLISHERS
        # ============================================================

        self.command_pub = self.create_publisher(
            Float64MultiArray,
            '/leg_position_controller/commands',
            10
        )

        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # ============================================================
        # TIME
        # ============================================================

        self.dt = (
            1.0 /
            self.control_frequency
        )

        self.t = 0.0

        self.start_time = (
            self.get_clock().now()
        )

        # ============================================================
        # SIMSCAPE / MATLAB CALIBRATION
        # ============================================================

        # ============================================================
        # SIMSCAPE / MATLAB CALIBRATION
        # ============================================================

        # Direcciones de eje en URDF (1.0 para lado izquierdo, -1.0 para derecho)
        self.coxa_axis_dir = np.array(
            [
                 1.0,
                -1.0,
                 1.0,
                -1.0
            ],
            dtype=float
        )

        self.S_vec = np.array(
            [
                 1.0,
                -1.0,
                 1.0,
                -1.0
            ],
            dtype=float
        )

        # ============================================================
        # TRAJECTORY SIGNS
        # ============================================================

        self.signo_y = np.array(
            [
                 1.0,
                -1.0,
                 1.0,
                -1.0
            ],
            dtype=float
        )

        self.signo_z = 1.0

        # ============================================================
        # GAIT PHASE OFFSETS
        # ============================================================

        # Original MATLAB / Simulink sequence:
        #
        # FL = 0.75 T
        # FR = 0.00 T
        # BL = 0.25 T
        # BR = 0.50 T
        #

        self.desfase = np.array(

            [
                0.75,
                0.00,
                0.25,
                0.50
            ],

            dtype=float

        ) * self.T

        # ============================================================
        # LINK LENGTHS
        # ============================================================

        self.L1 = 54.32
        self.L2 = 80.00
        self.L3 = 161.64

        # ============================================================
        # JOINT LIMITS
        # ============================================================

        # Límites provenientes del URDF.

        self.joint_limits = {
            'front_left': {
                'coxa': (-1.761120, 1.761120),
                'femur': (-1.570800, 1.570800),
                'tibia': (-1.570800, 1.570800)
            },
            'front_right': {
                'coxa': (-1.999850, 1.999850),
                'femur': (-1.570800, 1.570800),
                'tibia': (-1.570800, 1.570800)
            },
            'back_left': {
                'coxa': (-1.402110, 1.402110),
                'femur': (-1.570800, 1.570800),
                'tibia': (-1.570800, 1.570800)
            },
            'back_right': {
                'coxa': (-1.748240, 1.748240),
                'femur': (-1.570800, 1.570800),
                'tibia': (-1.570800, 1.570800)
            }
        }
        
        # ============================================================
        # STATE
        # ============================================================

        self.previous_command = np.zeros(
            12,
            dtype=float
        )

        self.last_debug_time = 0.0
        
        self.last_raw_clamp_debug_time = 0.0

        self.home_completed = False

        self.last_unreachable_warning = 0.0

        # ============================================================
        # TIMER
        # ============================================================

        self.timer = self.create_timer(

            self.dt,
            self.control_loop
        )

        # ============================================================
        # START MESSAGE
        # ============================================================

        self.get_logger().info(
            '=================================================='
        )

        self.get_logger().info(
            'AMTQ V2.0 GAIT CONTROLLER'
        )

        self.get_logger().info(
            'DAR + Elliptical Crawling Gait'
        )

        self.get_logger().info(
            '=================================================='
        )

        self.get_logger().info(
            f'Control frequency: '
            f'{self.control_frequency:.2f} Hz'
        )

        self.get_logger().info(
            f'Step length: '
            f'{self.L:.2f} mm'
        )

        self.get_logger().info(
            f'Step height: '
            f'{self.H:.2f} mm'
        )

        self.get_logger().info(
            f'Gait period: '
            f'{self.T:.2f} s'
        )

        self.get_logger().info(
            f'DAR iterations: '
            f'{self.N_DAR}'
        )

        self.get_logger().info(
            f'DAR lambda: '
            f'{self.lambda_val}'
        )

        self.get_logger().info(
            f'Gait enabled: '
            f'{self.gait_enabled}'
        )

        self.get_logger().info(
            'Internal leg order: '
            'FL -> FR -> BL -> BR'
        )

        self.get_logger().info(
            'Waiting for HOME phase...'
        )

    # ================================================================
    # DAR SPIN
    # ================================================================

    def dar_spin(
        self,
        r_ab,
        r_ac,
        axis
    ):

        eps_machine = np.finfo(
            float
        ).eps

        xab = r_ab[0]
        yab = r_ab[1]
        zab = r_ab[2]

        xac = r_ac[0]
        yac = r_ac[1]
        zac = r_ac[2]

        axis = axis.lower()

        if axis == 'x':

            value = (

                eps_machine
                +
                yab * zac
                -
                yac * zab
            )

        elif axis == 'y':

            value = (

                eps_machine
                -
                xab * zac
                +
                xac * zab
            )

        elif axis == 'z':

            value = (

                eps_machine
                +
                xab * yac
                -
                xac * yab
            )

        else:

            raise ValueError(

                f'Invalid axis: {axis}'
            )

        BIG = 1e8

        d = np.tanh(

            0.5
            *
            BIG
            *
            value
        )

        return d

    # ================================================================
    # DAR ANGLE
    # ================================================================

    def dar_angle(
        self,
        r_ab,
        r_ac,
        axis,
        method=3
    ):

        axis = axis.lower()

        # ------------------------------------------------------------
        # ELIMINATION VECTOR
        # ------------------------------------------------------------

        if axis == 'x':

            E = np.array(

                [
                    0.0,
                    1.0,
                    1.0
                ]
            )

            axis_vector = np.array(

                [
                    1.0,
                    0.0,
                    0.0
                ]
            )

        elif axis == 'y':

            E = np.array(

                [
                    1.0,
                    0.0,
                    1.0
                ]
            )

            axis_vector = np.array(

                [
                    0.0,
                    1.0,
                    0.0
                ]
            )

        elif axis == 'z':

            E = np.array(

                [
                    1.0,
                    1.0,
                    0.0
                ]
            )

            axis_vector = np.array(

                [
                    0.0,
                    0.0,
                    1.0
                ]
            )

        else:

            raise ValueError(

                f'Invalid axis: {axis}'
            )

        # ------------------------------------------------------------
        # HADAMARD PRODUCT
        # ------------------------------------------------------------

        v1 = E * r_ab
        v2 = E * r_ac

        norm_v1 = np.linalg.norm(
            v1
        )

        norm_v2 = np.linalg.norm(
            v2
        )

        if (

            norm_v1
            <
            np.finfo(float).eps

            or

            norm_v2
            <
            np.finfo(float).eps
        ):

            return 0.0

        # ------------------------------------------------------------
        # NORMALIZATION
        # ------------------------------------------------------------

        v1 = v1 / norm_v1
        v2 = v2 / norm_v2

        # ------------------------------------------------------------
        # DOT PRODUCT
        # ------------------------------------------------------------

        dot_product = np.dot(
            v1,
            v2
        )

        dot_product = np.clip(

            dot_product,
            -1.0,
             1.0
        )

        # ------------------------------------------------------------
        # CROSS PRODUCT
        # ------------------------------------------------------------

        cross_product = np.cross(
            v1,
            v2
        )

        cross_projection = np.dot(

            cross_product,
            axis_vector
        )

        # ------------------------------------------------------------
        # DAR EQUATION
        # ------------------------------------------------------------

        if method == 1:

            theta = np.arccos(
            dot_product
        )

        elif method == 2:

            value = np.clip(
                cross_projection,
                -1.0,
                1.0
            )

            theta = np.arcsin(
                value
            )

        else:

            theta = np.arctan2(
                cross_projection,
                dot_product
            )

            return theta

    # ================================================================
    # VALIDATE WORKSPACE
    # ================================================================

    def is_target_reachable(
        self,
        D
    ):

        minimum_distance = abs(
            self.L2
            -
            self.L3
        )

        maximum_distance = (

            self.L2
            +
            self.L3
        )

        return (

            minimum_distance
            <=
            D
            <=
            maximum_distance
        )

    # ================================================================
    # DAR INVERSE KINEMATICS
    # ================================================================

    def calculate_dar_ik(
        self,
        x,
        y,
        z
    ):

        q1 = np.zeros(
            4,
            dtype=float
        )

        q2 = np.zeros(
            4,
            dtype=float
        )

        q3 = np.zeros(
            4,
            dtype=float
        )

        for i in range(4):

            # --------------------------------------------------------
            # TARGET
            # --------------------------------------------------------

            r = np.array(

                [
                    x[i],
                    y[i],
                    z[i]
                ],

                dtype=float
            )

            # --------------------------------------------------------
            # HORIZONTAL PROJECTION
            # --------------------------------------------------------

            r_xy = np.linalg.norm(
                r[0:2]
            )

            R = (

                r_xy
                -
                self.L1
            )

            D = np.hypot(

                R,
                r[2]
            )

            # --------------------------------------------------------
            # NUMERICAL PROTECTION
            # --------------------------------------------------------

            if D < 1e-9:

                self.get_logger().warning(

                    f'Leg {i}: '
                    'Singular IK target detected.'
                )

                q1[i] = self.previous_command[
                    i * 3
                ]

                q2[i] = self.previous_command[
                    i * 3 + 1
                ]

                q3[i] = self.previous_command[
                    i * 3 + 2
                ]

                continue

            # --------------------------------------------------------
            # WORKSPACE VALIDATION
            # --------------------------------------------------------

            reachable = self.is_target_reachable(
                D
            )

            if not reachable:

                self.get_logger().warning(

                    f'Leg {i}: '
                    f'Target outside workspace. '
                    f'D={D:.3f}'
                )

            # ========================================================
            # COXA
            # ========================================================

            if r_xy < 1e-9:

                q1_geo = 0.0

            else:

                q1_geo = np.arctan2(

                    r[1],
                    r[0]
                )

            q1_dar = q1_geo

            coxa_current = np.array(

                [
                    self.L1
                    *
                    np.cos(q1_dar),

                    self.L1
                    *
                    np.sin(q1_dar),

                    0.0
                ]
            )

            coxa_target = np.array(

                [
                    r[0],
                    r[1],
                    0.0
                ]
            )

            # --------------------------------------------------------
            # DAR COXA
            # --------------------------------------------------------

            for _ in range(
                self.N_DAR
            ):

                dq1 = self.dar_angle(

                    coxa_current,
                    coxa_target,
                    'z',
                    3
                )

                q1_dar += (

                    self.lambda_val
                    *
                    dq1
                )

                coxa_current = np.array(

                    [
                        self.L1
                        *
                        np.cos(q1_dar),

                        self.L1
                        *
                        np.sin(q1_dar),

                        0.0
                    ]
                )

            # ========================================================
            # FEMUR
            # ========================================================

            alpha_raw = (

                self.L2 ** 2
                +
                D ** 2
                -
                self.L3 ** 2

            ) / (

                2.0
                *
                self.L2
                *
                D
            )

            alpha = np.clip(

                alpha_raw,
                -1.0,
                 1.0
            )

            gamma = np.arctan2(

                r[2],
                R
            )

            q2_geo = (

                gamma
                +
                np.arccos(alpha)
            )

            target_femur = np.array(

                [
                    R,
                    0.0,
                    r[2]
                ]
            )

            q2_dar = q2_geo

            # --------------------------------------------------------
            # DAR FEMUR
            # --------------------------------------------------------

            for _ in range(
                self.N_DAR
            ):

                knee_iter = np.array(

                    [
                        self.L2
                        *
                        np.cos(q2_dar),

                        0.0,

                        self.L2
                        *
                        np.sin(q2_dar)
                    ]
                )

                dq2 = self.dar_angle(

                    knee_iter,
                    target_femur,
                    'y',
                    3
                )

                q2_dar -= (

                    self.lambda_val
                    *
                    dq2
                )

            # ========================================================
            # TIBIA
            # ========================================================

            beta_raw = (

                self.L2 ** 2
                +
                self.L3 ** 2
                -
                D ** 2

            ) / (

                2.0
                *
                self.L2
                *
                self.L3
            )

            beta = np.clip(

                beta_raw,
                -1.0,
                 1.0
            )

            q3_geo = (

                np.pi
                -
                np.arccos(beta)
            )

            knee_position = np.array(

                [
                    self.L2
                    *
                    np.cos(q2_dar),

                    0.0,

                    self.L2
                    *
                    np.sin(q2_dar)
                ]
            )

            target_tibia = (

                target_femur
                -
                knee_position
            )

            q3_dar = q3_geo

            tibia_current = np.array(

                [
                    self.L3
                    *
                    np.cos(

                        q2_dar
                        -
                        q3_dar
                    ),

                    0.0,

                    self.L3
                    *
                    np.sin(

                        q2_dar
                        -
                        q3_dar
                    )
                ]
            )

            # --------------------------------------------------------
            # DAR TIBIA
            # --------------------------------------------------------

            for _ in range(
                self.N_DAR
            ):

                dq3 = self.dar_angle(

                    tibia_current,
                    target_tibia,
                    'y',
                    3
                )

                q3_dar -= (

                    self.lambda_val
                    *
                    dq3
                )

                tibia_current = np.array(

                    [
                        self.L3
                        *
                        np.cos(

                            q2_dar
                            -
                            q3_dar
                        ),

                        0.0,

                        self.L3
                        *
                        np.sin(

                            q2_dar
                            -
                            q3_dar
                        )
                    ]
                )

            # ========================================================
            # NORMALIZE ANGLES
            # ========================================================

            q1_out = np.arctan2(

                np.sin(q1_dar),
                np.cos(q1_dar)
            )

            q2_out = np.arctan2(

                np.sin(q2_dar),
                np.cos(q2_dar)
            )

            q3_out = np.arctan2(

                np.sin(q3_dar),
                np.cos(q3_dar)
            )

            # ========================================================
            # OFFSETS
            # ========================================================

            if self.home_test_coxa_zero and not self.gait_enabled:

                q1[i] = 0.0

            else:

                q1[i] = (
                    (q1_out * self.coxa_axis_dir[i])
                    -
                    self.S_vec[i] * 0.4636
                )

            q2[i] = (

                q2_out
                -
                0.1273
            )

            q3[i] = (

                q3_out
                -
                1.8338
            )

        return q1, q2, q3

    # ================================================================
    # ELLIPTICAL GAIT GENERATOR
    # ================================================================

    def generate_trajectory(
        self,
        t
    ):

        x = np.zeros(
            4,
            dtype=float
        )

        y = np.zeros(
            4,
            dtype=float
        )

        z = np.zeros(
            4,
            dtype=float
        )

        # ------------------------------------------------------------
        # GAIT TIMING
        # ------------------------------------------------------------

        T_swing = (

            self.T
            /
            4.0
        )

        T_stance = (

            3.0
            *
            self.T
            /
            4.0
        )

        for i in range(4):

            # --------------------------------------------------------
            # LOCAL CLOCK
            # --------------------------------------------------------

            t_effective = (

                t
                +
                self.desfase[i]
            )

            t_cycle = (

                t_effective
                %
                self.T
            )

            # ========================================================
            # SWING
            # ========================================================

            if t_cycle < T_swing:

                phi = (

                    np.pi
                    *
                    t_cycle
                    /
                    T_swing
                )

                paso = (

                    (
                        self.L
                        /
                        2.0
                    )

                    *
                    (
                        1.0
                        -
                        np.cos(phi)
                    )

                    -
                    (
                        self.L
                        /
                        2.0
                    )
                )
                
                if self.debug:
                    self.get_logger().info(
                        f'{self.leg_names[i]} | '
                        f'SWING | paso={paso:.2f}'
                    )

                z_offset = (

                    self.H
                    *
                    np.sin(phi)
                )

            # ========================================================
            # STANCE
            # ========================================================

            else:

                t_stance_local = (

                    t_cycle
                    -
                    T_swing
                )

                progress_stance = (

                    t_stance_local
                    /
                    T_stance
                )

                paso = (

                    self.L
                    /
                    2.0

                    -

                    (
                        self.L
                        *
                        progress_stance
                    )
                )
                
                if self.debug:
                    self.get_logger().info(
                        f'{self.leg_names[i]} | '
                        f'STANCE | paso={paso:.2f}'
                    )

                z_offset = 0.0

            # ========================================================
            # XYZ
            # ========================================================

            x[i] = (

                self.x_home
                +
                paso
                *
                0.707
            )

            y[i] = (

                self.y_home
                +
                paso
                *
                0.707
                *
                self.signo_y[i]
            )

            z[i] = (

                self.z_home
                +
                z_offset
                *
                self.signo_z
            )

        return x, y, z

    # ================================================================
    # JOINT LIMITS
    # ================================================================

    def clamp_joint_commands(self, q1, q2, q3):
            """Aplica los límites independientes de cada articulación según su pata."""
            leg_keys = ['front_left', 'front_right', 'back_left', 'back_right']
            q1_clamped = np.zeros(4)
            q2_clamped = np.zeros(4)
            q3_clamped = np.zeros(4)
    
            for i, leg in enumerate(leg_keys):
                lim = self.joint_limits[leg]
                q1_clamped[i] = np.clip(q1[i], lim['coxa'][0], lim['coxa'][1])
                q2_clamped[i] = np.clip(q2[i], lim['femur'][0], lim['femur'][1])
                q3_clamped[i] = np.clip(q3[i], lim['tibia'][0], lim['tibia'][1])
    
            return q1_clamped, q2_clamped, q3_clamped

    # ================================================================
    # BUILD COMMAND
    # ================================================================

    def build_command(
        self,
        q1,
        q2,
        q3
    ):

        command = []

        for i in range(4):

            command.extend(

                [

                    float(q1[i]),
                    float(q2[i]),
                    float(q3[i])
                ]
            )

        return np.array(

            command,
            dtype=float
        )

    # ================================================================
    # PUBLISH COMMAND
    # ================================================================

    def publish_command(
        self,
        command
    ):

        msg = Float64MultiArray()

        msg.data = command.tolist()

        self.command_pub.publish(
            msg
        )

        self.previous_command = np.copy(
            command
        )

    # ================================================================
    # PUBLISH JOINT STATES
    # ================================================================

    def publish_joint_states(
        self,
        q1,
        q2,
        q3
    ):

        if not self.publish_joint_states_enabled:

            return

        msg = JointState()

        msg.header.stamp = (

            self.get_clock()
            .now()
            .to_msg()
        )

        msg.name = self.joint_names

        positions = []

        for i in range(4):

            positions.extend(

                [

                    float(q1[i]),
                    float(q2[i]),
                    float(q3[i])
                ]
            )

        msg.position = positions

        self.joint_state_pub.publish(
            msg
        )

    # ================================================================
    # RAW / CLAMP DEBUG
    # ================================================================

    def print_raw_clamp_debug(
        self,
        q1_raw,
        q2_raw,
        q3_raw,
        q1,
        q2,
        q3
    ):

        if not self.debug:
            return

        self.get_logger().info(
            f'\n'
            f'=========== RAW vs CLAMP ===========\n'

            f'FL RAW   = '
            f'({q1_raw[0]:.4f}, '
            f'{q2_raw[0]:.4f}, '
            f'{q3_raw[0]:.4f})\n'

            f'FL CLAMP = '
            f'({q1[0]:.4f}, '
            f'{q2[0]:.4f}, '
            f'{q3[0]:.4f})\n'

            f'\n'

            f'FR RAW   = '
            f'({q1_raw[1]:.4f}, '
            f'{q2_raw[1]:.4f}, '
            f'{q3_raw[1]:.4f})\n'

            f'FR CLAMP = '
            f'({q1[1]:.4f}, '
            f'{q2[1]:.4f}, '
            f'{q3[1]:.4f})\n'

            f'\n'

            f'BL RAW   = '
            f'({q1_raw[2]:.4f}, '
            f'{q2_raw[2]:.4f}, '
            f'{q3_raw[2]:.4f})\n'

            f'BL CLAMP = '
            f'({q1[2]:.4f}, '
            f'{q2[2]:.4f}, '
            f'{q3[2]:.4f})\n'

            f'\n'

            f'BR RAW   = '
            f'({q1_raw[3]:.4f}, '
            f'{q2_raw[3]:.4f}, '
            f'{q3_raw[3]:.4f})\n'

            f'BR CLAMP = '
            f'({q1[3]:.4f}, '
            f'{q2[3]:.4f}, '
            f'{q3[3]:.4f})\n'

            f'===================================='
        )

    # ================================================================
    # DEBUG
    # ================================================================

    def print_debug_data(
        self,
        x,
        y,
        z,
        q1,
        q2,
        q3
    ):

        if not self.debug:

            return

        if (

            self.t
            -
            self.last_debug_time
            <
            1.0
        ):

            return

        self.last_debug_time = self.t

        self.get_logger().info(

            f'\n'

            f'================ DEBUG ================\n'

            f't = {self.t:.3f} s\n'

            f'\n'

            f'FL XYZ = '
            f'({x[0]:.2f}, '
            f'{y[0]:.2f}, '
            f'{z[0]:.2f})\n'

            f'FL Q = '
            f'({q1[0]:.4f}, '
            f'{q2[0]:.4f}, '
            f'{q3[0]:.4f})\n'

            f'\n'

            f'FR XYZ = '
            f'({x[1]:.2f}, '
            f'{y[1]:.2f}, '
            f'{z[1]:.2f})\n'

            f'FR Q = '
            f'({q1[1]:.4f}, '
            f'{q2[1]:.4f}, '
            f'{q3[1]:.4f})\n'

            f'\n'

            f'BL XYZ = '
            f'({x[2]:.2f}, '
            f'{y[2]:.2f}, '
            f'{z[2]:.2f})\n'

            f'BL Q = '
            f'({q1[2]:.4f}, '
            f'{q2[2]:.4f}, '
            f'{q3[2]:.4f})\n'

            f'\n'

            f'BR XYZ = '
            f'({x[3]:.2f}, '
            f'{y[3]:.2f}, '
            f'{z[3]:.2f})\n'

            f'BR Q = '
            f'({q1[3]:.4f}, '
            f'{q2[3]:.4f}, '
            f'{q3[3]:.4f})\n'

            f'======================================'
        )

    # ================================================================
    # CONTROL LOOP
    # ================================================================

    def control_loop(self):

        # ------------------------------------------------------------
        # CURRENT TIME
        # ------------------------------------------------------------

        elapsed = (

            self.get_clock().now()
            -
            self.start_time

        ).nanoseconds * 1e-9

        # ============================================================
        # PHASE 1: HOME
        # ============================================================

        if not self.gait_enabled:

            x = np.full(

                4,
                self.x_home
            )

            y = np.full(

                4,
                self.y_home
            )

            z = np.full(

                4,
                self.z_home
            )

            # --------------------------------------------------------
            # TRANSITION TO GAIT
            # --------------------------------------------------------

            if (

                elapsed
                >=
                self.home_duration
            ):

                self.gait_enabled = True

                self.home_completed = True

                self.t = 0.0

                self.get_logger().info(

                    'HOME completed. '
                    'Starting gait...'
                )

        # ============================================================
        # PHASE 2: GAIT
        # ============================================================

        else:

            x, y, z = self.generate_trajectory(
                self.t
            )

        # ============================================================
        # IK
        # ============================================================

        q1, q2, q3 = self.calculate_dar_ik(
            x,
            y,
            z
        )
        
        q1_raw = np.copy(q1)
        q2_raw = np.copy(q2)
        q3_raw = np.copy(q3)

        # ============================================================
        # JOINT LIMITS
        # ============================================================

        q1, q2, q3 = self.clamp_joint_commands(

            q1,
            q2,
            q3
        )
        
        # ============================================================
        # HOME RAW vs CLAMP DEBUG
        # ============================================================

        if not self.gait_enabled:

            self.get_logger().info(
                f'\n'
                f'=========== HOME IK ===========\n'

            f'FL RAW = '
            f'({q1_raw[0]:.4f}, '
            f'{q2_raw[0]:.4f}, '
            f'{q3_raw[0]:.4f}) | '

            f'CLAMP = '
            f'({q1[0]:.4f}, '
            f'{q2[0]:.4f}, '
            f'{q3[0]:.4f})\n'

            f'FR RAW = '
            f'({q1_raw[1]:.4f}, '
            f'{q2_raw[1]:.4f}, '
            f'{q3_raw[1]:.4f}) | '

            f'CLAMP = '
            f'({q1[1]:.4f}, '
            f'{q2[1]:.4f}, '
            f'{q3[1]:.4f})\n'

            f'BL RAW = '
            f'({q1_raw[2]:.4f}, '
            f'{q2_raw[2]:.4f}, '
            f'{q3_raw[2]:.4f}) | '

            f'CLAMP = '
            f'({q1[2]:.4f}, '
            f'{q2[2]:.4f}, '
            f'{q3[2]:.4f})\n'

            f'BR RAW = '
            f'({q1_raw[3]:.4f}, '
            f'{q2_raw[3]:.4f}, '
            f'{q3_raw[3]:.4f}) | '

            f'CLAMP = '
            f'({q1[3]:.4f}, '
            f'{q2[3]:.4f}, '
            f'{q3[3]:.4f})\n'

            f'================================'
        )
            
        if self.gait_enabled:
            self.print_raw_clamp_debug(
                q1_raw,
                q2_raw,
                q3_raw,
                q1,
                q2,
                q3
            )

        # ============================================================
        # BUILD COMMAND
        # ============================================================

        command = self.build_command(

            q1,
            q2,
            q3
        )

        # ============================================================
        # PUBLISH
        # ============================================================

        self.publish_command(
            command
        )

        self.publish_joint_states(

            q1,
            q2,
            q3
        )

        # ============================================================
        # DEBUG
        # ============================================================

        self.print_debug_data(

            x,
            y,
            z,

            q1,
            q2,
            q3
        )

        # ============================================================
        # ADVANCE GAIT TIME
        # ============================================================

        if self.gait_enabled:

            self.t += self.dt

            if self.t >= 100000.0:

                self.t = (

                    self.t
                    %
                    self.T
                )


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = AMTQGaitController()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        node.get_logger().info(

            'AMTQ gait controller stopped.'
        )

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()