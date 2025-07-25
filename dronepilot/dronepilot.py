from pymavlink import mavutil
import time
import threading
import sys 

class DroneController:
    """
    High-level wrapper around pymavlink for convenient drone control.
    Provides automatic connection, mode switching, arming, takeoff, and RC channel override.
    """

    def __init__(self, connection_string="tcp:127.0.0.1:5760", baud=57600):
        self.connection_string = connection_string
        self.baud = baud
        self.master = None
        self.rc_channels = [0] * 8  # 8 standard RC channels (initialized to 0, which means no override / 65535)
        self._lock = threading.Lock()
        self._connect()
        # Add a flag for graceful thread termination
        self._running = True 
        # Request extended status stream immediately after connection
        self._request_extended_status_stream()

    def _connect(self):
        """
        Establishes connection to the drone via MAVLink.
        """
        print(f"[INFO] Connecting to {self.connection_string}...")
        try:
            self.master = mavutil.mavlink_connection(self.connection_string, baud=self.baud)
            self.master.wait_heartbeat(timeout=10) # Increase timeout for heartbeat
            print(f"[INFO] Connected. System ID: {self.master.target_system}, Component ID: {self.master.target_component}")
        except Exception as e:
            raise ConnectionError(f"[ERROR] Failed to connect to drone: {e}")

    def _request_extended_status_stream(self):
        """
        Requests the MAV_DATA_STREAM_EXTENDED_STATUS to ensure STATUSTEXT messages are sent.
        """
        print("[INFO] Requesting MAV_DATA_STREAM_EXTENDED_STATUS (10 Hz)...")
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            10,  # Rate in Hz
            1    # Start sending
        )
        time.sleep(1) # Small delay to allow the drone to start sending data

    def arm(self):
        """
        Arms the drone with safety checks.
        """
        # Ensure we are in a mode that allows arming (e.g., ALT_HOLD, STABILIZE)
        # Note: GUIDED mode can also arm, but for this ALT_HOLD takeoff, we ensure ALT_HOLD first.
        self._ensure_mode("ALT_HOLD") 
        print("[INFO] Sending arm command...")
        
        # Clear any pending STATUSTEXT messages before arming attempt
        self.master.recv_match(type='STATUSTEXT', blocking=False, timeout=0.01)

        self.master.arducopter_arm()
        
        arm_start_time = time.time()
        timeout = 1 # Overall timeout for arming (increased for robustness)
        armed = False

        while time.time() - arm_start_time < timeout:
            if self.master.motors_armed():
                armed = True
                break
            
            # Continuously check for STATUSTEXT messages during the arming attempt
            msg = self.master.recv_match(type='STATUSTEXT', blocking=False, timeout=0.1)
            if msg:
                print(f"[PREARM-LIVE] {msg.text}")
                if "Arming failed" in msg.text or "PreArm" in msg.text:
                    print(f"[ERROR] Detected specific arming failure message: {msg.text}")
                    break # Break early if a clear failure message is seen
            time.sleep(0.05) # Small delay to avoid busy-waiting

        if armed:
            print("[INFO] Motors armed successfully.")
        else:
            print(f"[ERROR] Arming timed out after {timeout} seconds.")
            print("[ERROR] Checking for any lingering pre-arm failure messages:")
            self.print_prearm_checks(duration=3) # Print any available STATUSTEXT messages one last time
            raise RuntimeError("Failed to arm motors within the timeout period. Check drone logs/messages for pre-arm failures.")


    def disarm(self):
        """
        Disarms the drone.
        """
        print("[INFO] Sending disarm command...")
        self.master.arducopter_disarm()
        self._wait_for(lambda: not self.master.motors_armed(), "[INFO] Waiting for motors to disarm...", timeout=5)

    def _ensure_mode(self, mode_name):
        """
        Ensures the drone is in a specific mode, switching if necessary.
        Repeatedly sends the mode change command until the mode is confirmed
        or a timeout occurs.
        """
        print(f"[INFO] Current mode: {self.master.flightmode}")
        if self.master.flightmode == mode_name:
            print(f"[INFO] Drone is already in {mode_name} mode.")
            return

        print(f"[INFO] Attempting to switch to {mode_name} mode by repeatedly sending commands...")
        
        mode_switch_timeout = 15 # Overall timeout for mode switch (seconds)
        command_interval = 1.0   # How often to re-send the set_mode command (seconds)
        
        start_time = time.time()
        
        while self._running:
            # Check for overall timeout
            if time.time() - start_time > mode_switch_timeout:
                supported = self.master.mode_mapping()
                raise RuntimeError(
                    f"[ERROR] Timeout waiting for mode change to {mode_name}. "
                    f"Current mode: {self.master.flightmode}. "
                    f"Supported modes: {list(supported.keys())}"
                )

            # Send the set_mode command
            self.master.set_mode(mode_name)
            
            # Check if the mode has changed
            current_mode = self.master.flightmode
            print(f"[INFO] Sent set_mode to {mode_name}. Current mode: {current_mode}", end='\r')

            if current_mode == mode_name:
                print(f"\n[INFO] Successfully switched to {mode_name} mode.")
                return # Mode switch successful, exit the function

            # Wait for a short interval before re-sending the command and re-checking
            time.sleep(command_interval)

    def print_prearm_checks(self, duration=3):
        """
        Prints any STATUSTEXT messages that are currently in the buffer or arrive shortly.
        This is for debugging and does not block until "PreArm checks passed".
        Args:
            duration (int): How long to listen for messages in seconds.
        """
        print(f"[DEBUG] Printing available STATUSTEXT messages for {duration} seconds...")
        # Ensure stream is requested before attempting to read
        self._request_extended_status_stream()
        # Clear the message buffer before reading
        self.master.recv_match(type='STATUSTEXT', blocking=False, timeout=0.01) # Clear buffer
        
        end_time = time.time() + duration 
        while time.time() < end_time:
            msg = self.master.recv_match(type='STATUSTEXT', blocking=False, timeout=0.05) # Small timeout to not block too long
            if msg:
                print(f"[PREARM] {msg.text}")
            time.sleep(0.05) # Small delay
        print("[DEBUG] Finished printing available STATUSTEXT messages.")

    def set_rc_channels_to_neutral_for_arming(self):
        """
        Sets RC channels 1-4 to their neutral (1500) position and sends override.
        Throttle is set to minimum (1000) for safe arming.
        This is often required for arming to pass RC checks.
        """
        print("[INFO] Setting RC channels to neutral for arming (Roll=1500, Pitch=1500, Throttle=1000, Yaw=1500).")
        with self._lock:
            self.rc_channels[0] = 1500  # Roll Neutral
            self.rc_channels[1] = 1500  # Pitch Neutral
            self.rc_channels[2] = 1000  # Throttle Minimum for Arming
            self.rc_channels[3] = 1500  # Yaw Neutral
        self.send_rc_override()
        time.sleep(0.2) # Give drone a moment to receive these values

    def takeoff(self, altitude, climb_throttle=1600, hover_duration=0, takeoff_timeout=60.0):
        """
        Initiates takeoff to a specific altitude in meters using ALT_HOLD mode and RC override.
        The drone will arm, switch to ALT_HOLD, and then use throttle override to climb.

        Args:
            altitude (float): Target altitude in meters.
            climb_throttle (int): RC throttle value (e.g., 1600-1800) for climbing.
                                  1500 is neutral (hold altitude), 1000 is min, 2000 is max.
            hover_duration (float): Duration in seconds to hover at the target altitude after reaching it.
                                    Set to 0 for no hovering.
            takeoff_timeout (float): Maximum time in seconds to wait for takeoff to complete.
        """
        print(f"[INFO] Preparing for takeoff to {altitude} meters in ALT_HOLD mode...")
        
        # 1. Ensure drone is in ALT_HOLD mode
        self._ensure_mode("ALT_HOLD")
        
        # 2. Arm the motors if not already armed
        if not self.master.motors_armed():
            # Set RC channels to neutral/minimum before attempting to arm in ALT_HOLD
            # Throttle will be 1000 for arming.
            self.set_rc_channels_to_neutral_for_arming() 
            self.arm() # This will handle pre-arm checks and wait for arming
        else:
            print("[INFO] Motors are already armed.")

        # 3. Set RC channels for takeoff climb and begin monitoring altitude
        print(f"[INFO] Setting RC throttle to {climb_throttle} for climb and monitoring altitude...")
        
        # Initialize Roll, Pitch, Yaw to neutral (1500) for straight vertical climb
        # The throttle will be continuously updated within the loop.
        self.roll = 1500
        self.pitch = 1500
        self.yaw = 1500

        # Start time for overall takeoff timeout
        start_time = time.time()
        
        # Loop to climb until target altitude is reached
        while self._running:
            # Check for overall takeoff timeout
            if time.time() - start_time > takeoff_timeout:
                # IMPORTANT: If timeout occurs during climb, reduce throttle to neutral/disarm
                print(f"\n[ERROR] Takeoff timed out after {takeoff_timeout} seconds. Attempting to set throttle to neutral and disarm.")
                self.throttle = 1500 # Set to neutral
                self.disarm() # Attempt to disarm
                raise TimeoutError(f"[ERROR] Takeoff timed out after {takeoff_timeout} seconds while climbing.")
            
            # Continuously send the climb throttle command inside the loop
            # This ensures the drone maintains the climb input.
            self.throttle = climb_throttle # Re-sends the command on each iteration

            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.5) # Reduced timeout for quicker checks
            if msg:
                current_alt_m = msg.relative_alt / 1000.0  # millimeters to meters
                print(f"[INFO] Current altitude: {current_alt_m:.2f} m (Target: {altitude:.2f} m)", end='\r')

                # Check if target altitude is reached or exceeded
                if current_alt_m >= altitude:
                    print(f"\n[INFO] Target altitude {altitude:.2f} meters reached.")
                    break # Exit the climb loop
            else:
                # If no message, we still keep sending throttle, but log a warning
                print("[WARNING] No GLOBAL_POSITION_INT message received. Continuing climb...", end='\r')
            
            # Small delay to avoid busy-waiting and allow new messages to arrive
            time.sleep(0.05) # Slightly reduced sleep for faster response to altitude changes

        # 4. Once altitude is reached, set throttle back to neutral (1500) to hold altitude
        print(f"[INFO] Reached target altitude. Setting throttle to neutral (1500) to hold position.")
        self.throttle = 1500 # This will also call send_rc_override()
        time.sleep(1) # Give drone a moment to stabilize at new throttle

        # 5. Hover at the target altitude for the specified duration
        if hover_duration > 0:
            print(f"[INFO] Hovering at target altitude for {hover_duration} seconds...")
            # The throttle is already set to 1500 for holding altitude.
            # We simply wait for the specified duration.
            time.sleep(hover_duration)

    def _wait_for(self, condition_fn, message, timeout=10.0):
        """
        Utility to wait for a condition with timeout.
        """
        print(message)
        start = time.time()
        while self._running and not condition_fn(): # Add self._running check
            if time.time() - start > timeout:
                raise TimeoutError(f"[ERROR] Timeout waiting for condition: {message}")
            time.sleep(0.1)

    def _wait_for_altitude(self, target_alt, tolerance=0.5, timeout=60.0): # Increased timeout
        """
        Waits until the drone reaches the desired altitude within a tolerance.
        """
        print(f"[INFO] Waiting to reach altitude: {target_alt} meters...")
        start = time.time()
        while self._running: # Add self._running check
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
            if msg:
                alt_m = msg.relative_alt / 1000.0  # millimeters to meters
                print(f"[INFO] Current altitude: {alt_m:.2f} m", end='\r')
                if abs(alt_m - target_alt) <= tolerance:
                    print(f"\n[INFO] Target altitude {target_alt} meters reached.")
                    break
            if time.time() - start > timeout:
                raise TimeoutError("[ERROR] Timeout waiting for target altitude.")
            time.sleep(0.5)

    def send_rc_override(self):
        """
        Sends current RC override values to the drone.
        """
        with self._lock:
            # Value 0 in pymavlink for rc_channels_override_send means "no override" (65535 MAVLink)
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *[val if val != 0 else 65535 for val in self.rc_channels]
            )

    # RC getters and setters
    @property
    def roll(self):
        return self.rc_channels[0]

    @roll.setter
    def roll(self, value):
        self.rc_channels[0] = value
        self.send_rc_override()

    @property
    def pitch(self):
        return self.rc_channels[1]

    @pitch.setter
    def pitch(self, value):
        self.rc_channels[1] = value
        self.send_rc_override()

    @property
    def throttle(self):
        return self.rc_channels[2]

    @throttle.setter
    def throttle(self, value):
        self.rc_channels[2] = value
        self.send_rc_override()

    @property
    def yaw(self):
        return self.rc_channels[3]

    @yaw.setter
    def yaw(self, value):
        self.rc_channels[3] = value
        self.send_rc_override()

    def reset_rc_override(self):
        """
        Resets all RC overrides (sets to 0 so mavlink sends 65535, meaning no override).
        """
        print("[INFO] Resetting RC override.")
        with self._lock:
            self.rc_channels = [0] * 8
        self.send_rc_override()

    # New public method for mode change
    def set_mode(self, mode_name):
        """
        Sets the drone's flight mode.
        This is a public method that calls the internal _ensure_mode.
        """
        self._ensure_mode(mode_name)

    def stop(self):
        """
        Stops the drone controller, ensuring MAVLink connection is closed.
        """
        print("[INFO] Stopping DroneController...")
        self._running = False # Set the flag to False to stop wait loops
        if self.master:
            self.master.close()
            print("[INFO] MAVLink connection closed.")

# Update the example.py to use the new takeoff logic
if __name__ == '__main__':
    drone = None 
    try:
        drone = DroneController("tcp:127.0.0.1:5762", 57600) 

        drone.print_prearm_checks()

        # Call the new takeoff method with ALT_HOLD logic and specify hover duration
        # Take off to 5 meters using ALT_HOLD RC override and then hover for 5 seconds
        drone.takeoff(altitude=5, climb_throttle=1600, hover_duration=5) 
        
        # After takeoff and hovering, the drone is still in ALT_HOLD mode, maintaining altitude.
        # You can perform other actions here, or just let it hover until landing.
        
        # Land and disarm the motors
        drone.set_mode("LAND")
        drone.disarm()

    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt detected. Exiting gracefully.")
    except Exception as e:
        print(f"[ERROR] An unexpected error occurred: {e}")
    finally:
        if drone:
            drone.stop()
        print("[INFO] Program finished.")