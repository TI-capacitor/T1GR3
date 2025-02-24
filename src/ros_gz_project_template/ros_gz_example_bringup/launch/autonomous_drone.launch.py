import os
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Start the Autonomous Drone Python script as a ROS 2 node
        launch_ros.actions.Node(
            package='ros_gz_example_application',  # Ensure correct package name
            executable='autonomous_drone.py',
            name='autonomous_drone',
            output='screen',
            parameters=[{
                "use_sim_time": True
            }]
        ),

        # Bridge the Gazebo camera topic to ROS 2
        launch_ros.actions.Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            arguments=[
                "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image"
                "@sensor_msgs/msg/Image@gz.msgs.Image"
            ],
            output='screen'
        ),
    ])
