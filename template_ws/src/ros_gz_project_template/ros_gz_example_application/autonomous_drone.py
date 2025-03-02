import rclpy
from rclpy.node import Node
import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw

class AutonomousDrone(Node):
    def __init__(self):
        super().__init__('autonomous_drone')

        self.drone = System()
        self.target_marker_id = 4  # ArUco marker ID
        self.marker_found = False

        self.get_logger().info("Connecting to drone...")
        self.create_timer(1.0, self.run)

    async def connect_drone(self):
        """ Connect to PX4 and wait until connection is established. """
        self.get_logger().info("Waiting for MAVSDK to connect to PX4...")
        await self.drone.connect(system_address="udp://:14540")

        # Wait for the drone to connect
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("Drone connected!")
                break
            await asyncio.sleep(1)

        # Ensure home position is set before arming
        async for health in self.drone.telemetry.health():
            if health.is_home_position_ok:
                self.get_logger().info("Home position set!")
                break
            self.get_logger().info("Waiting for home position...")
            await asyncio.sleep(1)

        # Increase telemetry update rate
        await self.drone.telemetry.set_rate_position(10.0)
        await self.drone.telemetry.set_rate_altitude(10.0)

        self.get_logger().info("Arming the drone...")
        await self.drone.action.arm()
        self.get_logger().info("Armed successfully.")

        self.get_logger().info("Resetting Offboard Mode...")
        await self.drone.offboard.stop()
        await asyncio.sleep(1)

        self.get_logger().info("Starting Offboard Mode...")
        # Send multiple setpoints to ensure PX4 accepts Offboard mode
        for _ in range(10):
            await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))
            await asyncio.sleep(0.1)

        await self.drone.offboard.start()
        self.get_logger().info("Offboard mode started.")

    async def takeoff(self):
        """ Takeoff to a specified altitude. """
        self.get_logger().info("Taking off...")
        await self.drone.action.set_takeoff_altitude(10)
        await self.drone.action.takeoff()

        async for position in self.drone.telemetry.position():
            self.get_logger().info(f"Absolute: {position.absolute_altitude_m}, Relative: {position.relative_altitude_m}")
            if position.relative_altitude_m >= 9.5:
                self.get_logger().info("Reached takeoff altitude!")
                break
            await asyncio.sleep(1)

    async def search_pattern(self):
        """ Fly in a grid search pattern until the marker is detected """
        positions = [
            PositionNedYaw(20, 0, -5, 0), PositionNedYaw(20, 20, -5, 0),
            PositionNedYaw(0, 20, -5, 0), PositionNedYaw(-20, 20, -5, 0),
            PositionNedYaw(-20, 0, -5, 0), PositionNedYaw(-20, -20, -5, 0),
            PositionNedYaw(0, -20, -5, 0), PositionNedYaw(20, -20, -5, 0)
        ]

        for pos in positions:
            if self.marker_found:
                break
            self.get_logger().info(f"Setting position to {pos}")
            await self.drone.offboard.set_position_ned(pos)
            self.get_logger().info(f"Flying to {pos}")
            await asyncio.sleep(6)  # Ensure movement completion

    async def precision_land(self):
        """ Perform precision landing when marker is found. """
        self.get_logger().info("Landing on marker...")
        await self.drone.action.land()

        async for in_air in self.drone.telemetry.in_air():
            if not in_air:
                self.get_logger().info("Landed successfully!")
                break
            await asyncio.sleep(1)

    async def run(self):
        """ Main control loop. """
        await self.connect_drone()
        await self.takeoff()
        await self.search_pattern()
        await self.precision_land()

async def main(args=None):
    """ Properly handle ROS 2 with asyncio. """
    rclpy.init(args=args)
    node = AutonomousDrone()
    await node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
