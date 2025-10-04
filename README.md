# hsl_kuka

This repository contains setup instructions and usage notes for operating the **KUKA LBR iiwa 14 R820** robot with **ROS 2 Humble** via the [lbr_fri_ros2_stack](https://github.com/lbr-stack/lbr_fri_ros2_stack), as used in the **Hybrid Systems Lab**.

### Robot Software

* Download: [Google Drive link](https://drive.google.com/drive/folders/1SqIc5ExvVY7ObdWGkBGb_l8qLj4cPYb9?usp=sharing)

### Specifications

* **Robot:** KUKA LBR iiwa 14 R820 (RAL 9016)
* **Sunrise Workbench:**  v1.10
* **FRI:** v1.10

---

## 1. Environment Setup

### Install Miniforge

```bash
wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
```

### Setup RoboStack ROS 2 Humble

```bash
conda create -n ros_env -c conda-forge -c robostack-humble ros-humble-desktop
conda activate ros_env
conda install -c conda-forge compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep
```

---

## 2. Build the ROS Workspace

```bash
git clone https://github.com/ian-chuang/hsl_kuka.git
cd hsl_kuka/lbr-stack/
conda activate ros_env
rosdep install --from-paths src -i -r -y
colcon build
source install/setup.bash
```

---

## 3. Running the Robot

1. **Power On the Controller**

   * Switch on the control box and wait for the teach pendant to fully boot.
   * Start in **T1 Mode** for safety and testing (you can use **Auto** later once programs are verified).

2. **Confirm Network Connection**

   * Host IP should be set to **`172.31.1.148`** (can be configured as a static IP with subnet `255.255.255.0`).
   * Verify connection to the robot with:

     ```bash
     ping 172.31.1.147
     ```

3. **Start the Robot Application**

   * On the pendant, navigate to:
     `Applications → LBRServer`
   * Hold down the **back button** and the **green play button** simultaneously.

4. **Configure Remote Settings**

   * FRI send period: **10 ms**
   * IP address: **172.31.1.148**
   * FRI control mode: **POSITION_CONTROL**
   * FRI client command mode: **POSITION**

5. **Launch the ROS 2 Driver**

   ```bash
   conda activate ros_env
   ros2 launch lbr_bringup hardware.launch.py \
       ctrl:=lbr_joint_position_command_controller \
       model:=iiwa14
   ```

6. **Run a Demo in Another Terminal**

   ```bash
   conda activate ros_env
   source install/setup.bash
   ros2 run lbr_demos_cpp joint_sine_overlay --ros-args -r __ns:=/lbr
   ```

⚠️ **Always remain near the emergency stop button.** Rotate to reset if triggered.

---

## 4. Debugging

* KUKA ROS 2 Packages Documentation: [lbr_fri_ros2_stack](https://github.com/lbr-stack/lbr_fri_ros2_stack)
* Manuals: see [`help/`](./help/)
* KUKA Robot Software: [Google Drive link](https://drive.google.com/drive/folders/1SqIc5ExvVY7ObdWGkBGb_l8qLj4cPYb9?usp=sharing)
* For general robot setup instructions try [help/quickstart.pdf](./help/quickstart.pdf)

### Modes

The robot operating mode is selected by turning the **key switch** on the teach pendant, then pressing one of the mode buttons (**T1, T2, Aut**). When finished, turn the key switch back to return to the **home screen**.

* **T1 (Test Mode 1 – Manual):**

  * Safe mode for testing.
  * To move the robot, you must **hold down** the back button (small gray button or long trigger button).
  * While holding, you can:

    * Move joints manually with the `+` / `–` buttons.
    * Run a program by first selecting one in Applications dropdown and holding the **green play button** (either the arrow button or back green button).

* **T2 (Test Mode 2):**

  * I don't use. Consult the manual if needed.

* **Auto Mode:**

  * Runs programs **without needing to hold down any button**.
  * Use extreme caution—programs will execute automatically when the **green play button** is pressed.
  * Always verify your program works correctly in **T1 mode** before switching to Auto.

### Mastering Joints

* In case joints need to be mastered (will show warning on teach pendant)
* Go to **Robots tab → LBR iiwa 14 → Mastering**.
* Master all joints before running.

### Updating Robot Software and KUKA Support

* Use **Sunrise Workbench** if the robot software is corrupted or if you need to change system settings (e.g., robot IP, install or update software).
* Required software downloads (Sunrise Workbench + FRI): [Google Drive link](https://drive.google.com/drive/folders/1SqIc5ExvVY7ObdWGkBGb_l8qLj4cPYb9?usp=sharing)
* For KUKA support create an account at [my.kuka.com](https://my.kuka.com/) with a **UC Berkeley email**.
* Register under location **Cory 337** to be added to the robot’s support group and gain access to KUKA support.

### Sunrise Workbench Tips

* Follow [setup instructions](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_fri_ros2_stack/doc/hardware_setup.html) to properly setup Sunrise Workbench. (Requires Windows.)

#### Installing New Software

1. In **Sunrise Workbench**, go to:
   `Help → Install New Software → Add... → Archive...`

   * Select the `.zip` file for the software you want to install (e.g., FRI).

2. If you encounter errors (e.g., trying to add the same software twice):

   * Remove the entry under **Available Software Sites**, then re-add it.

3. Open `StationSetup.cat → Software` and add the software you want to install.

   * Then install it under **Installation**.

4. Once the robot restarts, **synchronize the project** (button to the left of the red debug icon near the top-left).

5. Reactivate safety on the robot teach pendant:

   * `Safety → Activation → Activate`
   * Password: **`argus`**

#### Helpful Java Applications

1. In **Sunrise Workbench**, go to:
   `File → New → Other... → Sunrise Folder → Application`

2. Add the following applications:

   * *Position and GMS Referencing of LBR iiwa*
   * *Braketest of LBR iiwa*

3. **Synchronize the project** to load the apps onto the teach pendant.

4. Run the applications on the pendant:

   * The *Position and GMS Referencing* app can help clear certain joint-related warnings.

#### Safety Configuration

⚠️ **Warning:** The following configuration was found to remove warnings but has **not been formally validated for safety**. Use with caution and consult official KUKA documentation before relying on it.

Steps:

1. Open `SafetyConfiguration.sconf`.
2. Remove all **custom PSM settings** by setting them to `None`.
3. Set the **Reaction Column** to `Port 1` — the corresponding row should then disappear.
4. Save and synchronize the project and restart robot.
5. Reactivate safety on the pendant:
   * `Safety → Activation → Activate`
   * Password: **`argus`**

### Network Debugging

#### Allow UDP Port for FRI

If you encounter firewall issues, open the FRI port (`30200/udp`) with:

```bash
sudo ufw allow 30200/udp
sudo ufw reload
```

#### Debugging FRI Communication

You can verify that the controller is sending FRI packets by running this Python script while an FRI app is running on pendant:

```python
import socket

UDP_IP = "0.0.0.0"
UDP_PORT = 30200

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on {UDP_IP}:{UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(1024)
    print(f"Received {len(data)} bytes from {addr[0]}:{addr[1]}")
    print(f"Data: {data}")
```

#### Check Basic Connectivity

Confirm that your PC can reach the robot:

```bash
ping 172.31.1.147
```

💡 The robot’s IP address can be modified through **Sunrise Workbench** if needed.

## 7. Additional Resources

* [lbr_fri_ros2_stack issues](https://github.com/lbr-stack/lbr_fri_ros2_stack/issues/302)
