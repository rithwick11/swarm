import rclpy
from rclpy.node import Node
from msg import DroneCommand

class GCSNode(Node):
    def __init__(self):
        super().__init__('GCSNode')
        self.drone1_publisher = self.create_publisher(DroneCommand, '/drone1_commands', 10)
        self.drone2_publisher = self.create_publisher(DroneCommand, '/drone2_commands', 10)
        self.timer_send_commands = self.create_timer(1.0, self.send_commands)

    def send_commands_to_drones(self, drone1_cmd, drone2_cmd):
        drone1_msg = DroneCommand()
        drone1_msg.x = drone1_cmd[0]
        drone1_msg.y = drone1_cmd[1]
        drone1_msg.z = drone1_cmd[2]
        drone1_msg.vx = drone1_cmd[3]
        drone1_msg.vy = drone1_cmd[4]
        drone1_msg.vz = drone1_cmd[5]
        self.drone1_publisher.publish(drone1_msg)

        drone2_msg = DroneCommand()
        drone2_msg.x = drone2_cmd[0]
        drone2_msg.y = drone2_cmd[1]
        drone2_msg.z = drone2_cmd[2]
        drone2_msg.vx = drone2_cmd[3]
        drone2_msg.vy = drone2_cmd[4]
        drone2_msg.vz = drone2_cmd[5]
        self.drone2_publisher.publish(drone2_msg)

def main(args=None):
    rclpy.init(args=args)
    print("Starting GCSNode...\n")
    gcs_node = GCSNode()

    # Example commands to be sent to drones [x, y, z, vx, vy, vz]
    drone1_cmd = [0.0, 0.0, 5.0, 0.0, 0.0, 0.0]
    drone2_cmd = [0.0, 0.0, 5.0, 0.0, 0.0, 0.0]

    # Send commands to drones
    gcs_node.send_commands_to_drones(drone1_cmd, drone2_cmd)

    rclpy.spin(gcs_node)
    gcs_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

