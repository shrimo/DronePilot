# DronePilot (In development)

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

drone = None 
try:
    drone = DroneController("tcp:127.0.0.1:5762", 57600) 

    drone.print_prearm_checks()

    # Call the new takeoff method with ALT_HOLD logic and specify hover duration
    # Take off to 5 meters using ALT_HOLD RC override and then hover for 5 seconds
    drone.set_mode("ALT_HOLD")
    drone.takeoff(altitude=5, climb_throttle=1900, hover_duration=5) 
    
    # After takeoff and hovering, the drone is still in ALT_HOLD mode, maintaining altitude.
    # You can perform other actions here, or just let it hover until landing.

    drone.disarm()

except KeyboardInterrupt:
    print("\n[INFO] KeyboardInterrupt detected. Exiting gracefully.")
except Exception as e:
    print(f"[ERROR] An unexpected error occurred: {e}")
finally:
    if drone:
        drone.stop()
    print("[INFO] Program finished.")
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

