import rclpy
from rclpy.node import Node
import asyncio
import numpy as np
import cv2
from mavsdk import System
from mavsdk.offboard import Offboard, PositionNedYaw, VelocityNedYaw


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

        # Wait until the drone is connected
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("Drone connected!")
                break
            await asyncio.sleep(1)

        self.get_logger().info("Arming the drone...")
        await self.drone.action.arm()
        self.get_logger().info("Armed successfully.")

        self.get_logger().info("Starting Offboard Mode...")
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))
        await self.drone.offboard.start()
        self.get_logger().info("Offboard mode started.")

    async def takeoff(self):
        """ Takeoff to a specified altitude. """
        self.get_logger().info("Taking off...")
        await self.drone.action.set_takeoff_altitude(5.0)  # 5m altitude
        await self.drone.action.takeoff()

        async for position in self.drone.telemetry.position():
            if position.relative_altitude_m >= 4.8:  # Slight buffer for takeoff success
                self.get_logger().info("Reached takeoff altitude!")
                break
            await asyncio.sleep(1)

    async def search_pattern(self):
        """ Fly in a grid search pattern until the marker is detected """
        positions = [
            PositionNedYaw(10, 0, -5, 0), PositionNedYaw(10, 10, -5, 0),
            PositionNedYaw(0, 10, -5, 0), PositionNedYaw(-10, 10, -5, 0),
            PositionNedYaw(-10, 0, -5, 0), PositionNedYaw(-10, -10, -5, 0),
            PositionNedYaw(0, -10, -5, 0), PositionNedYaw(10, -10, -5, 0)
        ]

        for pos in positions:
            if self.marker_found:
                break
            await self.drone.offboard.set_position_ned(pos)
            self.get_logger().info(f"Flying to {pos}")
            await asyncio.sleep(2)

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
