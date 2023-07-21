import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from swarm_communication.msg import DroneCommand
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


class Drone1Node(Node):
    def __init__(self):
        super().__init__('Drone1Node')
        self.command_publisher = self.create_publisher(DroneCommand, '/drone1_commands', 10)

        self.offboard_control_mode_publisher_1 = self.create_publisher(OffboardControlMode, "/px4_1/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher_1 = self.create_publisher(TrajectorySetpoint, "/px4_1/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher_1 = self.create_publisher(VehicleCommand, "/px4_1/fmu/in/vehicle_command", 10)

        self.offboard_setpoint_counter_ = 0
        self.mission_state = 0
        self.current_waypoint = [0.0, 0.0, -5.0]

        self.timer_offboard = self.create_timer(0.02, self.timer_offboard_cb)
        self.timer_mission = self.create_timer(8, self.mission)

    def mission(self):
        if self.mission_state == 0:
            """ Arming and takeoff to 1st waypoint """
            self.get_logger().info("Mission started")
            self.publish_vehicle_command_1(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            self.publish_vehicle_command_2(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            self.arm()
            self.get_logger().info("Vehicle armed")
            # Imposta il primo waypoint
            self.get_logger().info("First waypoint")
            self.current_waypoint = [0.0, 0.0, -5.0]
            self.mission_state = 1

        elif self.mission_state == 1:
            """Waypoint 2"""
            self.get_logger().info("Second waypoint")
            # Imposta secondo waypoint
            self.current_waypoint = [2.0, 2.0, -4.0]
            self.mission_state = 2

        elif self.mission_state == 2:
            """Landing"""
            self.get_logger().info("Landing request")
            self.publish_vehicle_command_1(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.publish_vehicle_command_2(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.mission_state = 3
        
        elif self.mission_state == 3:
            self.timer_offboard.cancel()
            self.get_logger().info("Mission finished")
            self.timer_mission.cancel()
            exit()

    def timer_offboard_cb(self):
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

    # Arm the vehicle
    def arm(self):
        self.publish_vehicle_command_1(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.publish_vehicle_command_2(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    # Disarm the vehicle
    def disarm(self):
        self.publish_vehicle_command_1(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")

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
        self.trajectory_setpoint_publisher_1.publish(msg)

    def publish_vehicle_command_1(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 2
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # time in microseconds
        self.vehicle_command_publisher_1.publish(msg)

    def send_command_to_drone2(self, x, y, z, vx, vy, vz):
        command_msg = DroneCommand()
        command_msg.x = x
        command_msg.y = y
        command_msg.z = z
        command_msg.vx = vx
        command_msg.vy = vy
        command_msg.vz = vz
        self.command_publisher.publish(command_msg)


def main(args=None):
    rclpy.init(args=args)
    print("Starting Drone1Node...\n")
    drone1_node = Drone1Node()
    rclpy.spin(drone1_node)
    drone1_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
