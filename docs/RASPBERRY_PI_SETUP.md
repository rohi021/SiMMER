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
