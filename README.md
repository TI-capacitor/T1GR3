# T1GR3
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
cd ~/template_ws
source env/bin/activate
```

---

## **2️⃣ Source Your ROS 2 Installation**
### **For ROS 2 Jazzy (Ubuntu 24.04)**
```bash
source /opt/ros/jazzy/setup.bash
```

### **For Other ROS 2 Distros (e.g., Humble, Rolling, etc.)**
```bash
source /opt/ros/<your-distro>/setup.bash
```
Replace `<your-distro>` with your actual ROS 2 version.

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

