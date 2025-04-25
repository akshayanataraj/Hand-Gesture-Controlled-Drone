import time
import serial
from pymavlink import mavutil

# === CONFIGURATION ===
PIXHAWK_PORT = 'COM9'    # Update this!
PIXHAWK_BAUD = 57600
ARDUINO_PORT = 'COM7'    # Update this!
ARDUINO_BAUD = 115200

# === CONNECTION FUNCTIONS ===
def connect_to_pixhawk():
    print("Connecting to Pixhawk...")
    mav = mavutil.mavlink_connection(PIXHAWK_PORT, baud=PIXHAWK_BAUD)
    mav.wait_heartbeat()
    print("Heartbeat received. Connected!")
    return mav

def connect_to_arduino():
    try:
        print("Connecting to Arduino...")
        arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
        print("Connected to Arduino!")
        return arduino
    except Exception as e:
        print(f"Failed to connect to Arduino: {e}")
        return None

# === RC OVERRIDE ===
def send_rc_override(mav, roll, pitch, throttle):
    mav.mav.rc_channels_override_send(
        mav.target_system,
        mav.target_component,
        roll,      # RC1 - Roll
        pitch,     # RC2 - Pitch
        throttle,  # RC3 - Throttle
        1500,      # RC4 - Yaw (neutral)
        65535, 65535, 65535, 65535  # RC5 to RC8 not overridden
    )

# === ARMING ===
def arm_drone(mav):
    print("Arming drone...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    mav.motors_armed_wait()
    print("Drone is armed!")

def disarm_drone(mav):
    print("Disarming drone...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    mav.motors_disarmed_wait()
    print("Drone is disarmed!")

# === READ FROM ARDUINO ===
def read_inputs_from_arduino(arduino):
    try:
        # Clear any lingering buffer to avoid stale data
        if arduino.in_waiting:
            arduino.reset_input_buffer()

        line = arduino.readline().decode('utf-8').strip()
        print(f"[DEBUG] Raw Arduino Line: {line}")
        parts = line.split(',')
        if len(parts) == 3:
            throttle = int(parts[0])
            roll = int(parts[1])
            pitch = int(parts[2])
            return throttle, roll, pitch
        else:
            print("Invalid data format")
            return None
    except Exception as e:
        print(f"Read error: {e}")
        return None

# === MAIN LOOP ===
def main():
    mav = connect_to_pixhawk()
    arduino = connect_to_arduino()

    print("Priming RC override...")
    for _ in range(20):
        send_rc_override(mav, 1500, 1500, 1000)  # neutral + min throttle
        time.sleep(0.02)
    
    arm_drone(mav)

    time.sleep(1)  # Allow time for the drone to stabilize

    print("Starting RC override. Ctrl+C to exit.")
    try:
        while True:
            values = read_inputs_from_arduino(arduino)
            if values:
                throttle, roll, pitch = values
                send_rc_override(mav, roll, pitch, throttle)
                print(f"Throttle: {throttle}, Roll: {roll}, Pitch: {pitch}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Exiting...")
        disarm_drone(mav)

    finally:
        send_rc_override(mav, 0, 0, 0)
        if arduino:
            arduino.close()

if __name__ == "__main__":
    main()