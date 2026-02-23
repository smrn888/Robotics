[Robotics_README.md](https://github.com/user-attachments/files/25480140/Robotics_README.md)
#  Robotics Projects

A collection of robotics and automation projects covering simulation, motion planning, computer vision, and industrial communication protocols.

---

## 📂 Projects

### 01 — CODESYS PLC + OPC UA + PyBullet Robot Simulation
> **Status:** ✅ Complete

A full Industry 4.0 stack connecting a real CODESYS PLC to a simulated Kuka IIWA robot arm. The robot uses computer vision to detect colored boxes and sort them into matching baskets using RRT+Spline trajectory planning.

**Stack:** `Python` `PyBullet` `CODESYS` `OPC UA` `OpenCV` `RRT` `Scipy`

**Highlights:**
- Real PLC ↔ Python communication via OPC UA (~1ms latency)
- HSV color detection with PyBullet ground-truth fallback
- RRT path planning + Cubic Spline smoothing + Trapezoid velocity profile
- 100% sort accuracy across all test rounds (7/7 boxes per cycle)

📁 [`01_CODESYS-OPC-UA-PyBullet/`](./01_CODESYS-OPC-UA-PyBullet/)

---

*More projects coming soon...*

---

## 🛠️ Common Stack

| Area | Tools |
|------|-------|
| Simulation | PyBullet, Gazebo |
| Motion Planning | RRT, MoveIt |
| Vision | OpenCV, PyTorch |
| Industrial Comms | OPC UA, EtherCAT, Modbus |
| Languages | Python, C++ |

---

**Author:** Moein
