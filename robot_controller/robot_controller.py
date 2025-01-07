import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publisher per inviare comandi di velocità
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer per inviare comandi di movimento ogni 0.5 secondi
        self.timer = self.create_timer(0.5, self.move_robot)
        self.get_logger().info("RobotController node has started!")

        # Stato del movimento (puoi personalizzarlo)
        self.is_moving_forward = True
        self.start_time = time.time()

    def move_robot(self):
        # Crea il messaggio Twist
        msg = Twist()

        # Movimento alternato avanti e rotazione
        elapsed_time = time.time() - self.start_time
        if elapsed_time < 5:  # Muovi avanti per 5 secondi
            msg.linear.x = 0.5  # Velocità lineare (m/s)
            msg.angular.z = 0.0  # Nessuna rotazione
            self.get_logger().info("Moving forward")
        elif elapsed_time < 10:  # Ruota per 5 secondi
            msg.linear.x = 0.0  # Nessun movimento lineare
            msg.angular.z = 0.5  # Rotazione (rad/s)
            self.get_logger().info("Rotating")
        else:
            self.start_time = time.time()  # Resetta il timer

        # Pubblica il comando
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

