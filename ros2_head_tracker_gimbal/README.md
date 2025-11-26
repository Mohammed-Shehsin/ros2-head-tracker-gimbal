# ROS2 Head Tracker Gimbal – Real-Time Face-Tracking Gimbal (URDF + RViz + ESP32 WiFi Control)

This repository contains a complete ROS2-based head-tracking gimbal system integrating:

- **Custom URDF/XACRO 2-DOF gimbal model**
- **RViz2 visualization**
- **ROS2 → ESP32 WiFi UDP servo control**
- **Physical MG90S / MG996R gimbal**
- **Real-time face-tracking using OpenCV + MediaPipe**

The project creates a full end-to-end pipeline:

**Simulation → ROS2 → UDP → ESP32 → Servos → Physical gimbal**

---

## 📦 Repository Structure
```
ros2_head_tracker_gimbal/
│
├── ros2/
│   ├── gimbal_description/
│   └── gimbal_udp_bridge/
│
├── esp32/
│   └── pinch_to_servo_udp_sta.ino
│
├── media/
│   ├── demo.gif
│   ├── wiring.png
│   └── model_render.png
│
└── README.md
```

---

## 🔧 1. Gimbal URDF Description (`gimbal_description`)

A realistic 2-axis gimbal consisting of:

- **Yaw joint** (bottom servo)
- **Pitch joint** (top servo)
- **Base plate**
- **Support column**
- **Camera mounting plate**

Main file:

src/gimbal_description/urdf/gimbal.urdf.xacro

### Launch RViz:

ros2 launch gimbal_description display.launch.py

This opens:

- The URDF model in RViz  
- `joint_state_publisher_gui` for real-time control

---

## 🌐 2. ESP32 WiFi + UDP Servo Controller

The ESP32 receives servo angles via UDP:

<yaw_angle>,<pitch_angle>

Example:

45,120

### Servo Connections

| Function | ESP32 Pin |
|---------|-----------|
| Yaw servo | GPIO 18 |
| Pitch servo | GPIO 19 |
| PWM freq | 50 Hz |
| Pulse width | 500–2400 µs |

Upload:

esp32/pinch_to_servo_udp_sta.ino

---

## 🧠 3. ROS2 → UDP Bridge (`gimbal_udp_bridge`)

This node:

1. Subscribes to `/joint_states`
2. Converts radians → degrees
3. Sends angles to ESP32 via UDP

### Run the bridge:

ros2 run gimbal_udp_bridge joint_to_udp –ros-args -p esp_ip:=

Example:

ros2 run gimbal_udp_bridge joint_to_udp –ros-args -p esp_ip:=192.168.1.22

You should see:

[UDP] Sent: 90,120

---

## 🎥 4. Face Tracking (Python + MediaPipe)

A Python script (added soon) performs:

- Face landmark detection  
- Nose + boundary vector estimation  
- Head orientation → yaw/pitch  
- Publishes `/joint_states` in real time  

This simultaneously drives:

- RViz simulation  
- Physical gimbal through the ESP32  

---

## 🛠 5. Build Instructions

cd ~/ros2_head_tracker_gimbal_ws
rm -rf build install log
colcon build
source install/setup.bash

Verify packages:

ros2 pkg list | grep gimbal

Expected:

gimbal_description
gimbal_udp_bridge

---

## 🧪 6. Full Pipeline Test

### Step 1 — Power ESP32 + Servos  
Use external **5–6V supply** (never power from USB!).  
Common GND required.

### Step 2 — Start RViz

ros2 launch gimbal_description display.launch.py

### Step 3 — Start UDP Bridge

ros2 run gimbal_udp_bridge joint_to_udp –ros-args -p esp_ip:=192.168.1.22

### Step 4 — Move sliders in RViz  
The physical gimbal should follow instantly.

---

## ⚙ Hardware Used

- ESP32 DevKit V1
- MG90S / MG996R servos
- External 6V supply
- Custom gimbal frame
- Ubuntu 22.04
- ROS2 Humble
- RViz2 + joint_state_publisher_gui

---

## 📌 Notes

- Always use separate servo power  
- Connect grounds together  
- UDP is fast but not guaranteed (acceptable for servo control)  
- Extra smoothing/PID can be added

---

## 📜 License

MIT License

---

## 👤 Author

**Mohammed Shehsin**  
Robotics & Automation Engineer  
Poznań University of Technology  
GitHub: https://github.com/Mohammed-Shehsin

If you find this project helpful, please ⭐ the repository.
