# example.py

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