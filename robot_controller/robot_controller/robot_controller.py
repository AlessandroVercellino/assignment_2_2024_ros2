import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publisher per inviare comandi di velocità
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Robot Controller node started!")
        self.get_logger().info("Enter the linear and angular velocities to control the robot.")

    def move_robot(self):
        while rclpy.ok():
            try:
                # velocità
                linear_x = float(input("Enter linear velocity (m/s): "))
                angular_z = float(input("Enter angular velocity (rad/s): "))

                # Twist
                cmd_vel = Twist()
                cmd_vel.linear.x = linear_x
                cmd_vel.angular.z = angular_z

                # Command Pubblisher 
                self.cmd_vel_publisher.publish(cmd_vel)
                self.get_logger().info(
                    f"Published velocities - Linear: {linear_x:.2f} m/s, Angular: {angular_z:.2f} rad/s"
                )

                # Tempo per lasciare al robot di eseguire il comando
                self.get_logger().info("Robot is moving... Press Ctrl+C to stop or input new velocities.")
            except ValueError:
                self.get_logger().error("Invalid input! Please enter numeric values.")

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()

    try:
        node.move_robot()
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



