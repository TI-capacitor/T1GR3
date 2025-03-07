import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import asyncio
from mavsdk import System
from cv_bridge import CvBridge
import cv2
import numpy as np

class PrecisionLanding(Node):
    def __init__(self):
        super().__init__('precision_landing')
        self.drone = System()
        self.bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.tag_detected = False

    async def connect_to_drone(self):
        await self.drone.connect(system_address="udp://:14540")
        self.get_logger().info("Waiting for drone...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("Drone connected!")
                break

    async def land_on_tag(self):
        """ Interrupts the mission and initiates precision landing """
        if not self.tag_detected:
            return
        
        self.get_logger().info("Tag detected! Interrupting mission and landing...")
        
        # Change to Position mode
        await self.drone.action.hold()
        await asyncio.sleep(1)

        # Start landing sequence
        await self.drone.action.land()

    def image_callback(self, msg):
        """ Processes camera feed to detect the tag """
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect ArUco (AprilTag can also be used)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        if ids is not None:
            self.get_logger().info("Tag Detected! Preparing to land...")
            self.tag_detected = True
            asyncio.ensure_future(self.land_on_tag())

def main(args=None):
    rclpy.init(args=args)
    precision_landing = PrecisionLanding()
    asyncio.run(precision_landing.connect_to_drone())
    rclpy.spin(precision_landing)
    precision_landing.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
