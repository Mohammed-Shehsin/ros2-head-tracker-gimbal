# 🤖 ROS2 Head Tracker Gimbal  
Dual-Servo Pan–Tilt System Controlled by Head Direction Tracking (OpenCV + Mediapipe)

This repository contains the complete implementation of a **2-DOF dual-servo gimbal** that can be controlled in real time using **head direction tracking** from a webcam or external vision system.  
The project supports:

- Real-time head yaw/pitch detection  
- UDP communication between Python → Arduino  
- Servo-based pan–tilt control  
- ROS2 URDF/Xacro full model of the physical gimbal  
- RViz2 visualization  
- ROS2-ready package structure  

---

## 🖼️ System Overview

### Physical Gimbal  
A dual-servo aluminium gimbal made of:
- Base plate  
- Yaw servo (bottom)
- Vertical column
- Pitch servo (upper)
- Camera plate mount  

### ROS2 Model  
✔ Fully built URDF/Xacro  
✔ Two revolute joints: `yaw_joint`, `pitch_joint`  
✔ Visualized in RViz2  
✔ Dimensions based on real hardware

---

## 🎥 Demo (GIF)  
(Add your GIF here)

```
![demo](media/head_tracker_demo.gif)
```

---

# 🚀 Features

### 🔹 **1. Head Tracking (OpenCV + Mediapipe)**
- Face detection
- Landmark extraction (eyes, nose)
- Direction vectors
- Continuous yaw & pitch score  
- Roll compensation

### 🔹 **2. 2-Servo Pan–Tilt Control**
- Servo1 = Yaw (left–right)
- Servo2 = Pitch (up–down)
- 90° center mapping
- Direction-to-servo mapping with smoothing & sensitivity tuning

### 🔹 **3. ROS2 Integration**
- URDF/Xacro robot model  
- RViz2 visualization  
- Joint State Publisher GUI  
- Ready for TF broadcasting  
- Ready for hardware bridge (Arduino or ESP32)

---

# ⚙️ Installation

### **Clone the repository**
```bash
git clone https://github.com/Mohammed-Shehsin/ros2-head-tracker-gimbal.git
cd ros2-head-tracker-gimbal
```

### **Python Dependencies**
Create `requirements.txt` (already included):

```
mediapipe
opencv-python
numpy
pyserial
```

Install:
```bash
pip install -r requirements.txt
```

---

# 🧠 Head Tracking – Mathematical Model

### **Yaw (Left–Right Rotation)**
$begin:math:display$
S\_\{\\text\{yaw\}\}
\=
\\frac\{d\_R \- d\_L\}\{d\_R \+ d\_L\}
$end:math:display$

- $begin:math:text$ d\_R $end:math:text$: Horizontal distance from nose to right eye  
- $begin:math:text$ d\_L $end:math:text$: Horizontal distance from nose to left eye  
- $begin:math:text$ S\_\{\\text\{yaw\}\} \> 0 $end:math:text$ → Turning Right  
- $begin:math:text$ S\_\{\\text\{yaw\}\} \< 0 $end:math:text$ → Turning Left  

---

### **Pitch (Up–Down Rotation)**

$begin:math:display$
S\_\{\\text\{pitch\}\}
\=
\\frac\{d\_D \- d\_U\}\{d\_D \+ d\_U\}
$end:math:display$

- $begin:math:text$ d\_U $end:math:text$: Vertical distance nose → upper face region  
- $begin:math:text$ d\_D $end:math:text$: Vertical distance nose → chin  
- $begin:math:text$ S\_\{\\text\{pitch\}\} \> 0 $end:math:text$ → Looking Up  
- $begin:math:text$ S\_\{\\text\{pitch\}\} \< 0 $end:math:text$ → Looking Down  

---

### **Roll Compensation (Head Tilt)**

$begin:math:display$
\\theta\_\{\\text\{roll\}\}
\=
\\arctan2\( y\_\{RE\} \- y\_\{LE\}\,\\\; x\_\{RE\} \- x\_\{LE\} \)
$end:math:display$

All facial points are rotated by $begin:math:text$ \-\\theta\_\{\\text\{roll\}\} $end:math:text$ to stabilize yaw/pitch output.

---

# 🛠️ Hardware Setup

### Components Used:
- MG90S micro servos ×2  
- Aluminium pan–tilt bracket  
- Arduino Nano / ESP32  
- External 5V servo supply  
- Web camera or USB cam  
- Optional ball-head for mounting  

### Wiring:
```
Arduino D3 → Servo1 (Yaw)
Arduino D5 → Servo2 (Pitch)
5V External → Servo Power
GND Shared between Arduino & Servos
```

---

# 📡 Communication (Python → Arduino)

### UDP Packet Format
```
<yaw_angle>,<pitch_angle>
```

Example:
```
120,80
```

### Arduino receives → moves servos accordingly.

---

# 🦾 ROS2 URDF/Xacro Model

### Package structure:
```
gimbal_description/
 ┣ urdf/
 ┃ ┗ gimbal.urdf.xacro
 ┣ rviz/
 ┃ ┗ display.rviz
 ┣ launch/
 ┃ ┗ display.launch.py
 ┗ package.xml
```

### Launch the model:
```bash
ros2 launch gimbal_description display.launch.py
```

To move joints:
```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

---

# 🧩 Project Structure

```
.
├── scripts/
│   └── head_tracker_udp.py
├── arduino/
│   └── dual_servo_udp.ino
├── gimbal_description/
│   ├── urdf/
│   ├── launch/
│   ├── rviz/
│   └── CMakeLists.txt
├── media/
│   └── demo.gif
└── README.md
```

---

# 📘 Roadmap

### ✅ Completed  
✔ Head tracking  
✔ Servo control  
✔ UDP communication  
✔ ROS2 URDF model  
✔ RViz visualization  

### 🔜 Next  
⬜ ROS2 Hardware Interface (ros2_control)  
⬜ TF broadcasting  
⬜ Camera mount CAD model  
⬜ PID-based servo smoothing  
⬜ Gazebo simulation  

---

# 🤝 Contributing  
Pull requests are welcome! Please open an issue first.

---

# 📄 License  
MIT License

---
