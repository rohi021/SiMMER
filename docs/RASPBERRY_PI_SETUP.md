# 🧾 Raspberry Pi Hotspot Connection — CMD Cheat Sheet

Quick-reference commands for connecting a laptop to the Raspberry Pi over a mobile hotspot and performing initial setup.

## 1. Find Your Laptop's Network Range

```cmd
ipconfig
```

Look for the **IPv4 Address** of your hotspot adapter (e.g. `10.158.167.x`).

## 2. Discover the Raspberry Pi on the Network

```cmd
for /L %i in (1,1,254) do @ping -n 1 -w 5 10.158.167.%i > nul
```

Pings every address in the subnet to force the Raspberry Pi to appear in the ARP table.

## 3. Identify the Raspberry Pi's IP

```cmd
arp -a
```

Find the Raspberry Pi's IP by matching its **MAC address** (usually starts with `b8:27:eb` or `dc:a6:32` for Pi 4).

## 4. SSH into the Raspberry Pi

```bash
ssh rohith@10.158.167.149
```

Replace the IP with the address found in step 3.

## 5. Verify the IP from Inside the Pi

```bash
hostname -I
```

## 6. Change the Raspberry Pi Password

```bash
passwd
```

## 7. Enable Interfaces (VNC, Camera, SSH, etc.)

```bash
sudo raspi-config
```

Navigate to **Interfacing Options** to enable VNC, Camera, SSH, and other peripherals.

## ✅ Final Confirmation

A successful login looks like:

```
rohith@raspberrypi:~ $
```

---

# 🧾 Hexapod ROS (Docker + Raspberry Pi) — Terminal Summary

## 1. Login to Raspberry Pi (from Laptop)

```bash
ssh rohith@<RASPBERRY_PI_IP>
```

Replace `<RASPBERRY_PI_IP>` with the IP discovered via `arp -a`.

## 2. Launch the ROS Docker Container

```bash
docker start <container_name>
docker exec -it <container_name> bash
```

## 3. Source the Workspace and Launch

```bash
source /catkin_ws/devel/setup.bash
roslaunch lmm_sc start.launch
```

## 4. Run Individual Nodes (Optional)

```bash
rosrun lmm_sc master.py              # Main motion planner
rosrun lmm_sc motor_controller.py    # Servo interface
rosrun lmm_sc joy_incremental.py     # Joystick teleoperation
```

---

> **Tip:** For simulation without hardware, use `roslaunch lmm_sc simulation.launch` instead of `start.launch`.

---

# 🎮 Joystick Setup on Raspberry Pi

Step-by-step commands to connect a USB/Bluetooth joystick and verify it works on the Raspberry Pi (run these in the Pi terminal or RealVNC session).

## 1. Install the Joystick Driver Package

```bash
sudo apt-get update
sudo apt-get install -y joystick
```

## 2. Load the Kernel Module

```bash
sudo modprobe joydev
```

This creates `/dev/input/js0` when a joystick is plugged in.

## 3. Connect the Joystick

- **USB (wired or wireless dongle):** Plug it into a Pi USB port.
- **Bluetooth:** Pair via `bluetoothctl`:

```bash
bluetoothctl
# Inside bluetoothctl:
power on
agent on
scan on
# Wait for your joystick MAC to appear, then:
pair XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
exit
```

## 4. Verify the Device Is Detected

```bash
ls /dev/input/js*
```

You should see `/dev/input/js0`. If not, check `dmesg | tail` for errors.

## 5. Quick Test with `jstest`

```bash
jstest /dev/input/js0
```

Move the sticks and press buttons — you should see axis/button values change in real time. Press `Ctrl+C` to stop.

## 6. Test with the SiMMER Script (No ROS Needed)

```bash
cd ~/catkin_ws/src/SiMMER/lmm_sc/src
python test_joystick.py
```

This script auto-detects the joystick, prints device info (name, axis count, button count), and shows live axis/button events. It also displays the axis mapping used by `joy_incremental.py` so you can verify or adjust the indices.

## 7. Test with ROS `joy_node`

Once you've confirmed the joystick works, test the full ROS path:

```bash
# Terminal 1 — start roscore
roscore

# Terminal 2 — launch the joy node
rosrun joy joy_node

# Terminal 3 — watch the /joy topic
rostopic echo /joy
```

Move the sticks to confirm axis values appear on the `/joy` topic.

## 8. Run the Full SiMMER Stack with Joystick

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch lmm_sc start.launch
```

This launches `joy_node`, `joy_incremental`, `motor_controller`, and all other nodes together. The joystick is now controlling the hexapod.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `/dev/input/js0` not found | Run `sudo modprobe joydev`, re-plug joystick |
| `Permission denied` on `/dev/input/js0` | Run `sudo chmod a+r /dev/input/js0` |
| Axes map differently than expected | Run `python test_joystick.py`, note which axis numbers move, then update `joy_incremental.py` lines 29-31 |
| Bluetooth joystick won't pair | Ensure `bluetooth` service is running: `sudo systemctl start bluetooth` |
| ROS `joy_node` error: "Couldn't open joystick" | Check device path: `rosparam set joy_node/dev "/dev/input/js0"` |
