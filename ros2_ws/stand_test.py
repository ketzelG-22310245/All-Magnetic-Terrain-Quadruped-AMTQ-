import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class QuadrupedStandTest(Node):
    def __init__(self):
        super().__init__('stand_test_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/leg_position_controller/commands', 10)
        self.get_logger().info('Nodo de prueba de postura iniciado.')

    def send_target_smooth(self, target_angles, duration=2.0, steps=50):
        """ Envía posiciones progresivamente para un movimiento fluido """
        delay = duration / steps
        
        # Asumiendo inicio desde ceros
        current_angles = [0.0] * len(target_angles)
        
        for i in range(1, steps + 1):
            alpha = i / float(steps)
            interpolated = [
                curr + alpha * (targ - curr) 
                for curr, targ in zip(current_angles, target_angles)
            ]
            
            msg = Float64MultiArray()
            msg.data = interpolated
            self.publisher_.publish(msg)
            time.sleep(delay)

def main():
    rclpy.init()
    node = QuadrupedStandTest()
    
    # Esperar 2 segundos a que el publisher establezca conexión
    time.sleep(2.0)

    node.get_logger().info('Parando robot suavemente...')
    # Posición neutra (Stand)
    stand_pose = [0.0]*12
    node.send_target_smooth(stand_pose, duration=1.0)
    
    time.sleep(1.0)

    node.get_logger().info('Levantando pata delantera izquierda...')
    # [FL_coxa, FL_femur, FL_tibia, FR_coxa, ...]
    lift_fl = [
        0.0, 0.4, -0.2,  # FL: Levanta muslo y flexiona tibia
        0.0, 0.0, 0.0,   # FR
        0.0, 0.0, 0.0,   # BL
        0.0, 0.0, 0.0    # BR
    ]
    node.send_target_smooth(lift_fl, duration=2.0)

    time.sleep(2.0)

    node.get_logger().info('Regresando a postura neutra...')
    node.send_target_smooth(stand_pose, duration=2.0)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()