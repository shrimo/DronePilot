# DronePilot

**DronePilot** is a high-level Python wrapper over `pymavlink` that provides a simple and convenient API for controlling MAVLink-compatible drones. It handles connection, arming, takeoff, flight mode changes, and RC override through an easy-to-use interface.

---

## ✨ Features

- Automatic MAVLink connection and heartbeat handling
- Safe arming and disarming
- Takeoff to a specified altitude
- Change flight modes: `ALTHOLD`, `LOITER`, `LAND`
- RC override with getters and setters: `roll`, `pitch`, `throttle`, `yaw`
- Built-in safety checks and timeouts

---

## 🚀 Installation

```bash
pip install pymavlink
```

---

## 🧠 Usage Example

```python
from dronepilot import DroneController
import time

# Connect to the drone via serial port (e.g., Raspberry Pi UART)
drone = DroneController("/dev/ttyAMA0", 57600)

# Take off to 5 meters
drone.takeoff(5)

# Switch to LOITER mode
drone.set_mode("LOITER")

# Example RC override
drone.roll = 1500
drone.pitch = 1600
drone.throttle = 1400
drone.yaw = 1500

time.sleep(3)

# Reset RC control to autopilot
drone.reset_rc_override()

# Land and disarm
drone.set_mode("LAND")
drone.disarm()
```

---

## 📡 Supported Modes

- `GUIDED` (used internally for takeoff)
- `ALTHOLD`
- `LOITER`
- `LAND`

---

## 📁 Project Structure

```
dronepilot/
├── dronepilot.py        # Main DroneController class
└── __init__.py          # (optional) package initializer
README.md
```

---

## 🛠 Requirements

- Python 3.6+
- [pymavlink](https://github.com/ArduPilot/pymavlink)
- A MAVLink-compatible flight controller (e.g. ArduPilot, PX4)

---

## ⚠️ Disclaimer

**This library directly controls flight behavior. Use with caution.** Test in simulation or with safety measures in place.

---

## 📃 License

MIT License

