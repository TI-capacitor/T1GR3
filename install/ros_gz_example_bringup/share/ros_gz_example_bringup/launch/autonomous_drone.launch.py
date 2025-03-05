import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=["python3", os.path.join(os.path.expanduser("~"), "template_ws/src/ros_gz_project_template/ros_gz_example_application/autonomous_drone.py")],
            output="screen"
        )
    ])
