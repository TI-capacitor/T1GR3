#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import asyncio
import numpy as np
import cv2
from sensor_msgs.msg import Image
from mavsdk import System
from mavsdk.offboard import PositionNedYaw
import struct

class AutonomousDrone(Node):
    def __init__(self):
        super().__init__('autonomous_drone')

        # Initialize MAVSDK system
        self.drone = System()
        self.marker_found = False

        # Subscribe to ROS 2 image topic
        self.image_sub = self.create_subscription(
            Image,
            "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image",
            self.image_callback,
            10
        )

        self.get_logger().info("🚀 Initializing drone connection...")
        self.create_timer(1.0, self.run)

    async def connect_drone(self):
        """ Connects to PX4 and arms the drone """
        self.get_logger().info("🔄 Waiting for MAVSDK to connect to PX4...")
        await self.drone.connect(system_address="udp://:14540")

        # Wait for connection
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("✅ Drone connected!")
                break
            await asyncio.sleep(1)

        # Arm the drone
        self.get_logger().info("🔄 Arming the drone...")
        await self.drone.action.arm()
        self.get_logger().info("✅ Armed successfully.")

        # Start Offboard Mode
        self.get_logger().info("🔄 Starting Offboard Mode...")
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))
        await self.drone.offboard.start()
        self.get_logger().info("✅ Offboard mode started.")

    async def takeoff(self):
        """ Takeoff to a safe altitude. """
        self.get_logger().info("🚀 Taking off...")
        await self.drone.action.set_takeoff_altitude(5.0)  # 5m altitude
        await self.drone.action.takeoff()

        # Wait until the drone reaches altitude
        async for position in self.drone.telemetry.position():
            if position.relative_altitude_m >= 4.8:
                self.get_logger().info("✅ Reached takeoff altitude!")
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

        self.get_logger().info("🚀 Starting search pattern...")
        for pos in positions:
            if self.marker_found:
                self.get_logger().info("🛑 Marker detected! Stopping search pattern.")
                break

            self.get_logger().info(f"🛫 Moving to position: {pos}")
            await self.drone.offboard.set_position_ned(pos)
            await asyncio.sleep(3)

        self.get_logger().info("🔄 Search pattern completed.")

    async def precision_land(self):
        """ Land on the ArUco marker if detected. """
        if self.marker_found:
            self.get_logger().info("✅ Marker detected! Landing now...")
            await self.drone.action.land()
            async for in_air in self.drone.telemetry.in_air():
                if not in_air:
                    self.get_logger().info("🛬 Landed successfully!")
                    break
                await asyncio.sleep(1)
        else:
            self.get_logger().warn("❌ No marker detected. Holding position.")

    def image_callback(self, msg):
        """ Process image directly from ROS 2 message without `cv_bridge` """
        try:
            # Convert ROS 2 Image to OpenCV format
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            frame = img_data.reshape(msg.height, msg.width, -1)

            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters()

            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            if ids is not None:
                self.get_logger().info(f"✅ ArUco marker detected! IDs: {ids.tolist()}")
                self.marker_found = True

                # Draw marker detection overlay for debugging
                frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.imshow("Aruco Detection", frame)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"⚠️ Error processing image: {e}")

    async def run(self):
        """ Main execution loop """
        await self.connect_drone()
        await self.takeoff()
        await self.search_pattern()
        await self.precision_land()


async def main(args=None):
    """ Start ROS 2 with asyncio """
    rclpy.init(args=args)
    node = AutonomousDrone()
    await node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
