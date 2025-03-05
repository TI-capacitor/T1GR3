# T1GR3
# old flow. New flow near end of document
to launch this you need 2 terminals open
One should be in your PX4-Autopilot folder, and anotheer one inside your template_ws folder
in PX4-Autopilot

'''bash
make px4_sitl gz_x500_mono_cam_down

'''
In template_ws:

# 📌 ROS 2 Setup Guide for Running `ros2` Commands

If you encounter the error:

```
ros2: command not found
```

it means your ROS 2 environment is not sourced. Follow these steps to fix it:

---

## **1️⃣ Activate Your Virtual Environment**
Before running any ROS 2 commands, ensure you're inside your virtual environment:

```bash
cd template_ws
source env/bin/activate
```

---

## **2️⃣ Source Your ROS 2 Installation**
### **For ROS 2 Jazzy (Ubuntu 24.04)**
```bash
source /opt/ros/jazzy/setup.bash
```



---

## **3️⃣ Source Your ROS 2 Workspace**
If you’ve built your workspace (`template_ws`), you need to source it as well:

```bash
source ~/template_ws/install/setup.bash
```

---

## **4️⃣ Verify That ROS 2 is Available**
Run the following command to check if ROS 2 is correctly set up:

```bash
ros2 doctor
```

If everything is working correctly, you should see a summary of your ROS 2 environment.

---

## **5️⃣ Run Your ROS 2 Command Again**
Now, you should be able to launch your drone script:

```bash
ros2 launch ros_gz_example_bringup autonomous_drone.launch.py
```

🚀 **Your ROS 2 environment is now properly set up!**
 
# Original ros_gz_project_template readme below
modified template located in /src
A template project integrating ROS 2 and Gazebo simulator.

## Included packages

* `ros_gz_example_description` - holds the sdf description of the simulated system and any other assets.

* `ros_gz_example_gazebo` - holds gazebo specific code and configurations. Namely this is where systems end up.

* `ros_gz_example_application` - holds ros2 specific code and configurations.

* `ros_gz_example_bringup` - holds launch files and high level utilities.


## Install

For using the template with Gazebo Fortress switch to the `fortress` branch of this repository, otherwise use the default branch `main` for Gazebo Harmonic onwards.

### Requirements

1. Choose a ROS and Gazebo combination https://gazebosim.org/docs/latest/ros_installation

   Note: If you're using a specific and unsupported Gazebo version with ROS 2, you might need to set the `GZ_VERSION` environment variable, for example:

    ```bash
    export GZ_VERSION=harmonic
    ```
    Also need to build [`ros_gz`](https://github.com/gazebosim/ros_gz) and [`sdformat_urdf`](https://github.com/ros/sdformat_urdf) from source if binaries are not available for your chosen combination.

1. Install necessary tools

    ```bash
    sudo apt install python3-vcstool python3-colcon-common-extensions git wget
    ```

### Use as template
Directly `Use this template` and create your project repository on Github.

Or start by creating a workspace and cloning the template repository:

   ```bash
   mkdir -p ~/template_ws/src
   cd ~/template_ws/src
   git clone https://github.com/gazebosim/ros_gz_project_template.git
   ```

## Usage

1. Install dependencies

    ```bash
    cd ~/template_ws
    source /opt/ros/jazzy/setup.bash
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -i -y --rosdistro <ROS_DISTRO>
    ```

1. Build the project

    ```bash
    colcon build --cmake-args -DBUILD_TESTING=ON
    ```

1. Source the workspace

    ```bash
    . ~/T1GR3/template_ws/install/setup.sh
    ```

1. Launch the simulation

    ```bash
    ros2 launch ros_gz_example_bringup px4_gz_sim.launch.py
    ```

For a more detailed guide on using this template see [documentation](https://gazebosim.org/docs/latest/ros_gz_project_template_guide).

