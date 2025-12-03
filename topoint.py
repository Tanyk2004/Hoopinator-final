import serial
import time

# --- CONFIGURATION ---
PORT = '/dev/ttyACM0' # Check this on your Pi
BAUD = 115200

# Motor Speeds
SPEED_FAST = 200
SPEED_SLOW = 100
SPEED_TURN = 120

# PID Constants for Line Following
KP = 0.1  # Tune this
BASE_SPEED = 150

# State Enum
class States:
    DEAD_RECKONING = "DEAD_RECKONING"
    FIND_LINE = "FIND_LINE"
    FOLLOW_LINE_1 = "FOLLOW_LINE_1"     # First leg
    TURN_TO_CENTER = "TURN_TO_CENTER"
    FOLLOW_LINE_2 = "FOLLOW_LINE_2"     # Leg to center
    TURN_BACK = "TURN_BACK"
    DRIVE_HOME = "DRIVE_HOME"
    STOP = "STOP"

# --- GLOBAL VARIABLES ---
curr_state = States.DEAD_RECKONING
line_pos = 9500
is_cross = 0
dist_front = 999
dist_right = 999
start_time = time.time()
maneuver_timer = 0

def send_motors(ser, left, right):
    msg = f"<{int(left)},{int(right)}>\n"
    ser.write(msg.encode('utf-8'))

def main():
    global curr_state, line_pos, is_cross, dist_front, dist_right, maneuver_timer
    
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        ser.reset_input_buffer()
    except:
        print("Error: Could not open Serial Port")
        return

    print("Starting Robot Brain...")
    
    # Give Arduino time to reset
    time.sleep(2)

    while True:
        # 1. READ SENSORS
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line.startswith("<") and line.endswith(">"):
                    content = line[1:-1]
                    parts = content.split(',')
                    if len(parts) == 4:
                        line_pos = int(parts[0])
                        is_cross = int(parts[1])
                        dist_front = float(parts[2])
                        dist_right = float(parts[3])
            except Exception as e:
                pass # Ignore bad packets

        # 2. STATE MACHINE LOGIC
        left_cmd = 0
        right_cmd = 0
        now = time.time()

        if curr_state == States.DEAD_RECKONING:
            print(f"Dead Reckoning | Front: {dist_front}")
            # Logic: Drive forward until wall is close OR line found
            left_cmd = SPEED_SLOW
            right_cmd = SPEED_SLOW
            
            # Condition 1: Line Found
            if line_pos < 9000:
                print("Line Found!")
                curr_state = States.FOLLOW_LINE_1
                
            # Condition 2: Wall too close (Transition to find line sweep?)
            if dist_front < 20: 
                left_cmd = 0; right_cmd = 0 # Stop for safety

        elif curr_state == States.FOLLOW_LINE_1:
            # PID Logic
            error = line_pos - 3000
            correction = error * KP
            left_cmd = BASE_SPEED + correction
            right_cmd = BASE_SPEED - correction
            
            # Check Intersection
            if is_cross == 1 and (now - maneuver_timer > 2.0): # 2s debounce
                print("Intersection 1 Reached -> Turning Center")
                curr_state = States.TURN_TO_CENTER
                maneuver_timer = now

        elif curr_state == States.TURN_TO_CENTER:
            # Blind Turn Logic (Assuming Right Turn towards center)
            # Adjust signs for Left turn: Left=negative, Right=positive
            left_cmd = SPEED_TURN
            right_cmd = -SPEED_TURN
            
            # Turn for 0.6 seconds (TUNE THIS TIME)
            if now - maneuver_timer > 0.6:
                # Optional: Only stop turning if line is seen
                if line_pos < 9000:
                    curr_state = States.FOLLOW_LINE_2
                    maneuver_timer = now # Reset timer for debounce

        elif curr_state == States.FOLLOW_LINE_2:
            # PID Logic again
            error = line_pos - 3000
            correction = error * KP
            left_cmd = BASE_SPEED + correction
            right_cmd = BASE_SPEED - correction
            
            # Check Center Intersection
            if is_cross == 1 and (now - maneuver_timer > 2.0):
                print("Center Reached -> Turning Back")
                curr_state = States.TURN_BACK
                maneuver_timer = now

        elif curr_state == States.TURN_BACK:
            # Turn opposite direction (Left)
            left_cmd = -SPEED_TURN
            right_cmd = SPEED_TURN
            
            if now - maneuver_timer > 0.6: # TUNE THIS TIME
                 curr_state = States.DRIVE_HOME
                 maneuver_timer = now

        elif curr_state == States.DRIVE_HOME:
            # Drive straight forward
            left_cmd = SPEED_FAST
            right_cmd = SPEED_FAST
            
            if now - maneuver_timer > 2.0: # Drive for 2 seconds
                curr_state = States.STOP

        elif curr_state == States.STOP:
            left_cmd = 0
            right_cmd = 0

        # Safety Clamps
        left_cmd = max(-400, min(400, left_cmd))
        right_cmd = max(-400, min(400, right_cmd))

        # 3. SEND COMMANDS
        send_motors(ser, left_cmd, right_cmd)
        
        # Small sleep to prevent spamming Serial
        time.sleep(0.05)

if __name__ == "__main__":
    main()