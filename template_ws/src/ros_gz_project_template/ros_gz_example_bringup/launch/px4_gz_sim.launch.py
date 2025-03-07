import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    home_dir = os.path.expanduser("~")
    px4_dir = os.path.join(home_dir, "T1GR3/PX4-Autopilot")

    px4_run_command = [
        "bash", "-c",
        f"cd {px4_dir} && PX4_GZ_WORLD=aruco make px4_sitl gz_x500_mono_cam_down"
    ]

    return LaunchDescription([
        # Start PX4 SITL with specific world and model
        ExecuteProcess(
            cmd=px4_run_command,
            output="screen"
        ),

        # Run the autonomous drone Python script
        ExecuteProcess(
            cmd=["python3", os.path.join(home_dir, "T1GR3/template_ws/src/ros_gz_project_template/ros_gz_example_application/autonomous_drone.py")],
            output="screen"
        )
    ])
