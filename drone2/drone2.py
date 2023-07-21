import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from msg import DroneCommand
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class Drone2Node(Node):
    def __init__(self):
        super().__init__('Drone2Node')
        self.command_subscriber = self.create_subscription(DroneCommand, '/drone2_commands', self.command_callback, 10)

        self.offboard_control_mode_publisher_2 = self.create_publisher(OffboardControlMode, "/px4_2/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher_2 = self.create_publisher(TrajectorySetpoint, "/px4_2/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher_2 = self.create_publisher(VehicleCommand, "/px4_2/fmu/in/vehicle_command", 10)

        self.offboard_setpoint_counter_ = 0
        self.mission_state = 0
        self.current_waypoint = [0.0, 0.0, -5.0]

        self.timer_offboard = self.create_timer(0.02, self.timer_offboard_cb)
        self.timer_mission = self.create_timer(8, self.mission)

    def command_callback(self, msg):
        x = msg.x
        y = msg.y
        z = msg.z
        vx = msg.vx
        vy = msg.vy
        vz = msg.vz

        # Publish the desired trajectory setpoint to move drone2 to the specified position and velocity
        trajectory_setpoint_msg = TrajectorySetpoint()
        trajectory_setpoint_msg.position.x = x
        trajectory_setpoint_msg.position.y = y
        trajectory_setpoint_msg.position.z = z
        trajectory_setpoint_msg.velocity.x = vx
        trajectory_setpoint_msg.velocity.y = vy
        trajectory_setpoint_msg.velocity.z = vz

        # You can set other fields of the trajectory_setpoint_msg as required
        trajectory_setpoint_msg.yaw = 0.0  # Set the desired yaw angle in radians

        self.trajectory_setpoint_publisher.publish(trajectory_setpoint_msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.offboard_control_mode_publisher_1.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = self.current_waypoint
        msg.yaw = -3.14  # [-PI:PI]
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.trajectory_setpoint_publisher_2.publish(msg)

    def publish_vehicle_command_2(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 3
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.vehicle_command_publisher_2.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    print("Starting Drone2Node...\n")
    drone2_node = Drone2Node()
    rclpy.spin(drone2_node)
    drone2_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
