# Import necessary modules
import socket
import xml.etree.ElementTree as ET
import threading
import time
import numpy as np
import datetime
import logging

# Set up logging
logging.basicConfig(
    filename=f'kuka_rsi_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}.log',
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# Global variables for target positions (desired positions for the robot)
target_positions = {
    "X": 0.0,
    "Y": 0.0,
    "Z": 0.0,
    "A": 0.0,
    "B": 0.0,
    "C": 0.0
}

# Current robot positions (updated from robot feedback
current_positions = {
    "X": 0.0,
    "Y": 0.0,
    "Z": 0.0,
    "A": 0.0,
    "B": 0.0,
    "C": 0.0
}

# Correction values for manual adjustments
correction_val = {
    "X": 0.0,
    "Y": 0.0,
    "Z": 0.0,
    "A": 0.0,
    "B": 0.0,
    "C": 0.0
}

# Initial correction positions (used for relative corrections)
initial_corr_pos ={
    "X": 0.0,
    "Y": 0.0,
    "Z": 0.0,   
    "A": 0.0,
    "B": 0.0,
    "C": 0.0
}

# Waypoint corrections - adjustments for each step
waypoint_corrections = {
    1: {"X": 0, "Y": 0, "Z": 0, "A": 0, "B": 0, "C": 0},      # No correction at waypoint 1
    2: {"X": 0, "Y": 0, "Z": 0, "A": 0, "B": 0, "C": 0},     # Move 20mm more in Y
    3: {"X": 0, "Y": 0, "Z": 0, "A": 0, "B": 0, "C": 0},    # Move 30mm more in -Z
    4: {"X": 0, "Y": 0, "Z": 0, "A": 0, "B": 0, "C": 0},  # Move -15mm in X and Y
    5: {"X": 0, "Y": 0, "Z": 0, "A": 0, "B": 0, "C": 0},      # No correction at home
    6: {"X": 0, "Y": 0, "Z": 0, "A": 0, "B": 0, "C": 0},      # MOVECORR
    7: {"X": 0, "Y": 0, "Z": 0, "A": 0, "B": 0, "C": 0},
    8: {"X": 0, "Y": 0, "Z": 0, "A": 0, "B": 0, "C": 0},
    9: {"X": 0, "Y": 0, "Z": 0, "A": 0, "B": 0, "C": 0}
}

# Flags and variables for program state
target_mode = True  # Indicates whether to use target positions

connected = False  # Connection status with the robot
last_packet_time = 0  # Timestamp of the last received packet
packet_counter = 0  # Counter for received packets
current_step = 0  # Current waypoint step
last_movecorr_flag = '0'  # Last MOVECORR flag state
running = True  # Flag to control the program's main loop
stop_flag = 0  # Flag to stop MOVECORR mode
routine_num = 99  # Routine number (99 means no routine is active)
routine_user = list()  # Queue for user-defined routines
routine_user_flag = 0  # Indicates if a routine is currently active

# function to convert the angles from [-180,180] to [0 360]
def convert_angle_to360(angle):
    return angle + 360 if angle < 0 else angle

# Function to normalize angles to the range [-180, 180]
def convert_angle_to180(angle):
    if angle > 180:
        return angle - 360
    elif angle < -180:
        return angle + 360
    else :
        return angle    
    
# Function to calculate the shortest angular difference
def shortest_angle_difference(target, current):
    # Calculate the raw difference
    
    diff = convert_angle_to360(target) - convert_angle_to360(current)
    
    if diff > 180.0:
        diff -= 360.0
    elif diff < -180.0:
        diff += 360.0
    
    
        
    return diff


# Function to calculate corrections for robot movement
def calculate_corrections():
    """Calculate corrections using proportional control for smoother movement."""
    
    
    corrections = {}
    position_gain = 0.005 # Gain for position corrections
    rotation_gain = 0.005 # Gain for rotation corrections
    max_correction_position = 0.05  # Maximum position correction
    max_correction_angle = 0.025 #Maximum rotation correction
    
    
            
    # Normal waypoint mode with lower gains for precise poitioning
    
    position_gain = 0.005  # Lower gain for waypoint mode
    rotation_gain = 0.005
    max_correction_position = 0.05  # Lower limits for waypoint mode
    max_correction_angle = 0.025
    
    
    # Check if a manual target has been set (non-zero values)
    has_manual_target = any(target_positions[axis] != 0.0 for axis in target_positions)
    
    for axis in ["X", "Y", "Z"]:
        if has_manual_target:
            # If manual target exists, combine it with waypoint correction
            combined_target = target_positions[axis] 
            diff = combined_target - current_positions[axis]
        else:
            # If no manual target, just apply waypoint correction directly
            diff = 0
            
        # Calculate proportional correction with limits
        corrections[axis] = max(-max_correction_position, min(max_correction_position, position_gain * diff))
    for axis in ["A", "B", "C"]:
        if has_manual_target:
            # For rotational axes with manual target
            combined_target = target_positions[axis] 
            diff = shortest_angle_difference(combined_target, current_positions[axis])
        else:
            # For rotational axes with only waypoint correction
            diff = shortest_angle_difference(current_positions[axis], current_positions[axis])
            
        if axis == "A":
            corrections[axis] = max(-max_correction_angle, min(max_correction_angle, rotation_gain * diff))
        else:
            corrections[axis] = -max(-max_correction_angle, min(max_correction_angle, rotation_gain * diff))

    return corrections



def correction_send():
    """Calculate corrections using proportional control for smoother movement."""
    global correction_val, initial_corr_pos, current_positions

    corrections = {}
    position_gain = 0.005 # Higher gain for MOVECORR mode
    rotation_gain = 0.005
    max_correction_position = 0.05  # Higher limits for MOVECORR mode
    max_correction_angle = 0.005
    
    
            
    
    # Normal waypoint mode with lower gains for precise positioning
    
    position_gain = 0.005  # Lower gain for waypoint mode
    rotation_gain = 0.005
    max_correction_position = 0.05  # Lower limits for waypoint mode
    max_correction_angle = 0.0025
    
    
    for axis in correction_val:
        if correction_val[axis] != 0.0:
            axes = axis

       
    for axis in correction_val:
        
        
        
        if axis == axes:
            if axis == "B" or axis == "C":
                target_positions[axis] = convert_angle_to180(correction_val[axis] + initial_corr_pos[axis])
                diff = shortest_angle_difference(target_positions[axis], current_positions[axis])
                corrections[axis] = -max(-max_correction_angle, min(max_correction_angle, rotation_gain * diff))
                
            elif axis == "A":
                target_positions[axis] = convert_angle_to180(correction_val[axis] + initial_corr_pos[axis])
                diff = shortest_angle_difference(target_positions[axis], current_positions[axis])
                corrections[axis] = max(-max_correction_angle, min(max_correction_angle, rotation_gain * diff))
            
            else:
                target_positions[axis] = correction_val[axis] + initial_corr_pos[axis]
                diff = target_positions[axis] - current_positions[axis]
                
                corrections[axis] = max(-max_correction_position, min(max_correction_position, position_gain * diff))
                
        else:
            corrections[axis] = 0 
    

    return corrections


# Function to generate XML response with calculated corrections
def create_response(ipoc, corrections):
    """Generate XML response with calculated corrections."""
    response = (
        f'<Sen Type="ImFree"><EStr>Path Controller</EStr>'
        f'<Tech T21="1.09" T22="2.08" T23="3.07" T24="4.06" T25="5.05" T26="6.04" T27="7.03" T28="8.02" T29="9.01" T210="10.00" />'
        f'<RKorr X="{corrections["X"]:.4f}" Y="{corrections["Y"]:.4f}" Z="{corrections["Z"]:.4f}" '
        f'A="{corrections["A"]:.4f}" B="{corrections["B"]:.4f}" C="{corrections["C"]:.4f}" />'
        f'<Routine>{routine_num}</Routine><DiO>{stop_flag}</DiO><IPOC>{ipoc}</IPOC></Sen>'
    )
    
    return response

# Function to handle user input in a separate thread
def user_input_thread():
    """Thread to handle user input for controlling the program."""
    global running, target_positions, target_mode, current_positions, initial_corr_pos, stop_flag, routine_num, routine_user_flag, routine_user
    
    print("\nCommand Interface for RSI Path Controller")
    print("----------------------------------------")
    print("Commands:")
    print("  target AXIS VALUE - Set single target position (e.g., target X 1500)")
    print("  (X,Y,Z,A,B,C) - Set all target positions at once (e.g., (2010,215,1700,180,90,90))")
    print("  correctio X 50 - Set the correction value for an axis directly")
    print("  routine 2 - Set the routine number to activate a routine ")
    print("  reset - Reset all target positions to zero")
    print("  waypoint - Show current waypoint and correction")
    print("  current - Show current robot position")
    print("  target - Show target positions")
    print("  correction - Show correction value")
    print("  status - Show connection status")
    print("  help - Show this help message")
    print("  exit - Exit the program")
    
    while running:
        cmd = input("> ").strip()
        
        if cmd.lower() == "exit":
            print("Exiting program...")
            running = False
            break
            
        elif cmd.lower() == "help":
            print("Commands:")
            print("  target AXIS VALUE - Set single target position (e.g., target X 1500)")
            print("  (X,Y,Z,A,B,C) - Set all target positions at once (e.g., (1500,200,800,0,90,0))")
            print("  correctio X 50 - Set the correction value for an axis directly")
            print("  routine 2 - Set the routinr number to activate a routine ")
            print("  reset - Reset all target positions to zero")
            print("  waypoint - Show current waypoint and correction")
            print("  current - Show current robot position")
            print("  target - Show target positions")
            print("  correction - Show correction value")
            print("  status - Show connection status")
            print("  help - Show this help message")
            print("  exit - Exit the program")
            
        elif cmd.lower() == "current":
            print("Current robot position:")
            for axis, value in current_positions.items():
                print(f"  {axis}: {value:.2f}")
                
        elif cmd.lower() == "stop":
            print("stopping movecorr.")
            stop_flag = 1
                
        elif cmd.lower() == "target":
            print("Target positions:")
            for axis, value in target_positions.items():
                print(f"  {axis}: {value:.2f}")

        elif cmd.lower() == "routine":
            print("routines in que:")
            print(routine_user)
        
        elif cmd.lower() == "reset":
            for axis in target_positions:
                target_positions[axis] = 0.0
            print("Reset all target positions to zero")
            logging.info("Reset all target positions to zero")
                
        elif cmd.lower() == "waypoint":
            print(f"Current waypoint: {current_step}")
            if current_step == 6:
                print("In MOVECORR mode (free movement)")
            if current_step in waypoint_corrections:
                print("Waypoint correction:")
                for axis, value in waypoint_corrections[current_step].items():
                    print(f"  {axis}: {value:.2f}")
            else:
                print("No corrections defined for this waypoint")
                
        elif cmd.lower() == "status":
            global connected, packet_counter, last_packet_time
            print(f"Connection status: {'Connected' if connected else 'Disconnected'}")
            print(f"Packets received: {packet_counter}")
            print(f"Current waypoint: {current_step}")
            
            if connected:
                print(f"Last packet received: {time.time() - last_packet_time:.1f} seconds ago")
        
        # Parse format like (X,Y,Z,A,B,C)
        elif cmd.startswith("(") and cmd.endswith(")"):
            try:
                target_mode = True
                # Extract values from parentheses
                values_str = cmd[1:-1]  # Remove the parentheses
                values = [float(val.strip()) for val in values_str.split(",")]
                
                # Check if we have exactly 6 values
                if len(values) == 6:
                    axes = ["X", "Y", "Z", "A", "B", "C"]
                    for i, axis in enumerate(axes):
                        target_positions[axis] = values[i]
                    
                    print(f"Set target position to: X={values[0]:.2f}, Y={values[1]:.2f}, Z={values[2]:.2f}, "
                          f"A={values[3]:.2f}, B={values[4]:.2f}, C={values[5]:.2f}")
                    logging.info(f"Set target position to: X={values[0]:.2f}, Y={values[1]:.2f}, Z={values[2]:.2f}, "
                               f"A={values[3]:.2f}, B={values[4]:.2f}, C={values[5]:.2f}")
                else:
                    print("Error: Format must include exactly 6 values for X,Y,Z,A,B,C")
            except ValueError:
                print("Error: Invalid format. Use (X,Y,Z,A,B,C) with numeric values")
        
        # Parse format like (correction X 50)        
        elif cmd.lower().startswith("correction"):
            parts = cmd.split()
            if len(parts) == 3 and parts[1].upper() in target_positions:
                try:
                    for axis in ["X", "Y", "Z", "A", "B", "C"]:
                        correction_val[axis] = 0.0
                        initial_corr_pos[axis] = current_positions[axis]
                    target_mode = False
                    axis = parts[1].upper()
                    value = float(parts[2])
                    correction_val[axis] = value
                    print(f"Set correction {axis} to {value:.2f}")
                    logging.info(f"Set correction {axis} to {value:.2f}")
                    
                except ValueError:
                    print("Error: Value must be a number")
            else:
                print("Error: Command format should be 'target AXIS VALUE'")
                print("Valid axes are: X, Y, Z, A, B, C")
         
        elif cmd.lower().startswith("routine"):
            parts = cmd.split()
            if len(parts) == 2:
                try:
                    routine_user.append(int(parts[1]))
                    
                    
                    print(f" {parts[1]} added to the que")
                    logging.info(f" {parts[1]} added to the que")
                except ValueError:
                    print("Error: Value must be a number")
            else:
                print("Error: Command format should be 'routine VALUE'")
                   
        else:
            print("Unknown command. Type 'help' for available commands.")

def communication_thread():
    """Thread to handle robot communication."""
    global running, connected, packet_counter, last_packet_time, current_positions 
    global current_step, stop_flag, last_movecorr_flag, routine_num, routine_user_flag, routine_user
    
    HOST, PORT = "10.10.10.20", 59152
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    sock.settimeout(0.5)  # Set timeout for socket operations
    
    print(f"Server listening on {HOST}:{PORT}")
    logging.info(f"Server listening on {HOST}:{PORT}")
    print("Waiting for robot connection...")
    
    try:
        while running:
            try:
                data, addr = sock.recvfrom(1024)  # Receive data from KUKA
                
                # Update connection status
                last_packet_time = time.time()
                packet_counter += 1
                
                if not connected:
                    print(f"Connection established with robot at {addr[0]}:{addr[1]}")
                    logging.info(f"Connection established with robot at {addr[0]}:{addr[1]}")
                    connected = True
                
                # Parse received XML
                received_xml = data.decode('utf-8')
                
                try:
                    root = ET.fromstring(received_xml)
                    
                    # Extract IPOC value
                    ipoc_elem = root.find(".//IPOC")
                    if ipoc_elem is not None:
                        ipoc = ipoc_elem.text
                    else:
                        ipoc = "0000000"
                        print("Warning: No IPOC found in received XML")
                    
                    # Extract current position
                    rist_elem = root.find(".//RIst")
                    if rist_elem is not None:
                        for axis in current_positions.keys():
                            if axis in rist_elem.attrib:
                                current_positions[axis] = float(rist_elem.get(axis, "0.0"))
                    
                    # Extract current step
                    step_elem = root.find(".//Step")
                    if step_elem is not None and step_elem.text:
                        new_step = int(step_elem.text)
                        if new_step != current_step:
                            print(f"Waypoint changed: {current_step} → {new_step}")
                            logging.info(f"Waypoint changed: {current_step} → {new_step}")
                            current_step = new_step
                            
                            # Special handling for MOVECORR mode (step 6)
                            if current_step == 6:
                                print("Entered MOVECORR mode - free movement available")
                                logging.info("Entered MOVECORR mode - free movement available")
                            
                            # Log the waypoint correction
                            if current_step in waypoint_corrections:
                                corr = waypoint_corrections[current_step]
                                corr_str = ", ".join([f"{axis}={val}" for axis, val in corr.items() if val != 0])
                                if corr_str:
                                    print(f"Waypoint {current_step} correction: {corr_str}")
                                    logging.info(f"Waypoint {current_step} correction: {corr_str}")
                                    
                    # Extract movecorr flag
                    movecorr_flag_elem = root.find(".//Movecorr_flag")
                    if movecorr_flag_elem is not None:
                        new_movecorr_flag = movecorr_flag_elem.text
                        if new_movecorr_flag == "1" and new_movecorr_flag != last_movecorr_flag:
                            last_movecorr_flag = new_movecorr_flag
                            print("MOVECORR is activated")
                            logging.info("MOVECORR is activated")
                        elif new_movecorr_flag == "0" and new_movecorr_flag != last_movecorr_flag:
                            last_movecorr_flag = new_movecorr_flag
                            print("MOVECORR is stoped")
                            logging.info("MOVECORR is stoped")
                    
                    # Extract AnOut_1 value
                    # This is used to set the routine number
                    an_out_elem = root.find(".//AnOut_1")
                    
                    if an_out_elem is not None:
                        # print(f"AnOut_1: {an_out_elem.text}")
                        
                        # If AnOut_1 is "99", the routine number is set to 99, but only if:
                        # - The current routine number is not already 99
                        # - No user-defined routine is currently active (routine_user_flag == 0)
                        if an_out_elem.text == "99" and routine_num != 99 and routine_user_flag == 0:
                            routine_num = 99
                            print("Routine number is set to 99")
                            logging.info("Routine number is set to 99")
                        # If there are user-defined routines in the queue and the current routine number is 99:
                        # - Pop the first routine from the queue and set it as the active routine
                        # - Set the routine_user_flag to 1 to indicate a user-defined routine is active# Extract AnOut_1 value  
                        # This is used to set the routine number based on the robot's output
                        elif len(routine_user) > 0 and routine_num == 99:
                            routine_num = routine_user.pop(0)
                            routine_user_flag = 1
                            
                            print(f"Routine number is set to {routine_num}")
                            logging.info("Routine number is set to {routine_num}")
                            
                        elif routine_user_flag == 1 and routine_num == int(an_out_elem.text):
                            routine_user_flag = 0
                            print(f"Routine flag is off")
                            logging.info("Routine flag is off")
                        
                    
                except ET.ParseError as e:
                    print(f"Error parsing XML: {e}")
                    logging.error(f"Error parsing XML: {e}")
                    continue
                
                # Calculate corrections based on current targets and waypoint
                if target_mode :
                    corrections = calculate_corrections()
                else: 
                    corrections = correction_send()
                
                # Create response XML with calculated corrections
                response_xml = create_response(ipoc, corrections)
                
                # Send response to robot
                sock.sendto(response_xml.encode('utf-8'), addr)
                
                # reset the stop flag
                if last_movecorr_flag == "0" and stop_flag == 1:
                    stop_flag = 0
                
                # Log status periodically
                if packet_counter % 100 == 0:
                    pos_str = ", ".join([f"{axis}={current_positions[axis]:.1f}" for axis in ["X", "Y", "Z", "A", "B", "C"]])
                    
                    print(f"Step {current_step}| Pos: {pos_str}")
                
            except socket.timeout:
                # Check if connection is lost
                if connected and time.time() - last_packet_time > 3.0:
                    print("Connection to robot seems lost. Waiting for reconnection...")
                    logging.warning("Connection to robot seems lost. Waiting for reconnection...")
                    connected = False
                continue
                
            except Exception as e:
                print(f"Error in communication: {e}")
                logging.error(f"Error in communication: {e}")
                time.sleep(0.5)  # Prevent CPU spinning on errors
                
    except Exception as e:
        print(f"Communication thread error: {e}")
        logging.error(f"Communication thread error: {e}")
    finally:
        sock.close()
        print("Socket closed")
        logging.info("Socket closed")

def main():
    """Main function."""
    global running
    
    print("KUKA RSI Path Controller")
    print("------------------------")
    print("This program applies corrections to KRL-defined paths")
    print("by tracking waypoint steps and applying specific offsets.")
    print("Step 6 enables MOVECORR mode for free movement.")
    print("\nWaypoint corrections:")
    for step, corr in waypoint_corrections.items():
        corr_str = ", ".join([f"{axis}={val}mm" for axis, val in corr.items() if val != 0])
        if step == 6:
            print(f"  Step {step}: MOVECORR mode (free movement)")
        elif corr_str:
            print(f"  Step {step}: {corr_str}")
        else:
            print(f"  Step {step}: No correction")
    
    # Create and start the communication thread
    comm_thread = threading.Thread(target=communication_thread)
    comm_thread.daemon = True
    comm_thread.start()
    
    # Create and start the user input thread
    input_thread = threading.Thread(target=user_input_thread)
    input_thread.daemon = True
    input_thread.start()
    
    # Wait for threads to complete
    try:
        while running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        logging.info("Program interrupted by user")
    finally:
        running = False
        print("Waiting for threads to terminate...")
        comm_thread.join(timeout=2.0)
        input_thread.join(timeout=2.0)
        print("Program terminated")
        logging.info("Program terminated")

if __name__ == "__main__":
    main()
