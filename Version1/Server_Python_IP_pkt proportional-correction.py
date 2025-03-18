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

# Global variables for target positions
target_positions = {
    "X": 0.0,
    "Y": 0.0,
    "Z": 0.0,
    "A": 0.0,
    "B": 0.0,
    "C": 0.0
}

# Current robot positions
current_positions = {
    "X": 0.0,
    "Y": 0.0,
    "Z": 0.0,
    "A": 0.0,
    "B": 0.0,
    "C": 0.0
}

# Connection status
connected = False
last_packet_time = 0
packet_counter = 0

# Flag to control the program
running = True

def shortest_angle_difference(target, current):
    """Calculate the shortest angle difference between target and current angles."""
    # Calculate the raw difference
    diff = target - current
    
    # Normalize the difference to be within the range -180 to 180
    if diff > 180:
        diff -= 360
    elif diff < -180:
        diff += 360
        
    return diff

# def calculate_corrections():
#     """Calculate corrections based on target and current positions."""
#     corrections = {}
    
#     # Calculate position differences (X, Y, Z)
#     for axis in ["X", "Y", "Z"]:
#         diff = target_positions[axis] - current_positions[axis]
#         # Apply small corrections in the right direction
#         if abs(diff) > 1:(2159,0,1480,180,-15,-180) 

#             corrections[axis] = 0.0256 if diff > 0 else -0.0256
#         else:
#             corrections[axis] = 0.0
    
#     # Calculate orientation differences (A, B, C) using shortest angle
#     for axis in ["A", "B", "C"]:
#         diff = shortest_angle_difference(target_positions[axis], current_positions[axis])
#         # Apply small corrections in the right direction
#         if abs(diff) > 1:
#             corrections[axis] = 0.0256 if diff > 0 else -0.0256
#         else:
#             corrections[axis] = 0.0
    
#     return corrections

def calculate_corrections():
    """Calculate corrections using proportional control for smoother movement."""
    corrections = {}
    gain = 0.01  # Correction factor (adjustable)

    for axis in ["X", "Y", "Z"]:
        diff = target_positions[axis] - current_positions[axis]
        corrections[axis] = max(-0.1, min(0.1, gain * diff))  # Limit corrections

    for axis in ["A", "B", "C"]:
        diff = shortest_angle_difference(target_positions[axis], current_positions[axis])
        corrections[axis] = max(-0.1, min(0.1, gain * diff))  # Limit corrections

    return corrections


def create_response(ipoc, corrections):
    """Generate XML response with calculated corrections."""
    response = (
        f'<Sen Type="ImFree"><EStr>Message from RSI TestServer</EStr>'
        f'<Tech T21="1.09" T22="2.08" T23="3.07" T24="4.06" T25="5.05" T26="6.04" T27="7.03" T28="8.02" T29="9.01" T210="10.00" />'
        f'<RKorr X="{corrections["X"]:.4f}" Y="{corrections["Y"]:.4f}" Z="{corrections["Z"]:.4f}" '
        f'A="{corrections["A"]:.4f}" B="{corrections["B"]:.4f}" C="{corrections["C"]:.4f}" />'
        f'<DiO>125</DiO><IPOC>{ipoc}</IPOC></Sen>'
    )
    return response

def user_input_thread():
    """Thread to handle user input for controlling the program."""
    global running, target_positions
    
    print("\nCommand Interface for RSI Controller")
    print("------------------------------------")
    print("Commands:")
    print("  target AXIS VALUE - Set single target position (e.g., target X 1500)")
    print("  (X,Y,Z,A,B,C) - Set all target positions at once (e.g., (1500,200,800,0,90,0))")
    print("  current - Show current robot position")
    print("  target - Show target positions")
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
            print("  current - Show current robot position")
            print("  target - Show target positions")
            print("  status - Show connection status")
            print("  help - Show this help message")
            print("  exit - Exit the program")
            
        elif cmd.lower() == "current":
            print("Current robot position:")
            for axis, value in current_positions.items():
                print(f"  {axis}: {value:.2f}")
                
        elif cmd.lower() == "target":
            print("Target positions:")
            for axis, value in target_positions.items():
                print(f"  {axis}: {value:.2f}")
                
        elif cmd.lower() == "status":
            global connected, packet_counter, last_packet_time
            print(f"Connection status: {'Connected' if connected else 'Disconnected'}")
            print(f"Packets received: {packet_counter}")
            if connected:
                print(f"Last packet received: {time.time() - last_packet_time:.1f} seconds ago")
        
        # Parse format like (X,Y,Z,A,B,C)
        elif cmd.startswith("(") and cmd.endswith(")"):
            try:
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
                
        elif cmd.lower().startswith("target "):
            parts = cmd.split()
            if len(parts) == 3 and parts[1].upper() in target_positions:
                try:
                    axis = parts[1].upper()
                    value = float(parts[2])
                    target_positions[axis] = value
                    print(f"Set target {axis} to {value:.2f}")
                    logging.info(f"Set target {axis} to {value:.2f}")
                except ValueError:
                    print("Error: Value must be a number")
            else:
                print("Error: Command format should be 'target AXIS VALUE'")
                print("Valid axes are: X, Y, Z, A, B, C")
                
        else:
            print("Unknown command. Type 'help' for available commands.")

def communication_thread():
    """Thread to handle robot communication."""
    global running, connected, packet_counter, last_packet_time, current_positions
    
    HOST, PORT = "10.10.10.20", 59152
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    sock.settimeout(0.5)  # Set timeout for socket operations
    
    print(f"Server listening on {HOST}:{PORT}")
    logging.info(f"Server listening on {HOST}:{PORT}")
    
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
                
                # Log every 500th packet to avoid excessive logging
                if packet_counter % 500 == 0:
                    print(f"Received packet #{packet_counter}")
                    logging.info(f"Received packet #{packet_counter}")
                
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
                        
                        # Log position periodically
                        if packet_counter % 500 == 0:
                            pos_str = ", ".join([f"{axis}={current_positions[axis]:.2f}" for axis in current_positions])
                            print(f"Current position: {pos_str}")
                            logging.info(f"Current position: {pos_str}")
                    
                except ET.ParseError as e:
                    print(f"Error parsing XML: {e}")
                    logging.error(f"Error parsing XML: {e}")
                    continue
                
                # Calculate corrections based on current targets
                corrections = calculate_corrections()
                
                # Create response XML with calculated corrections
                response_xml = create_response(ipoc, corrections)
                
                # Send response to robot
                sock.sendto(response_xml.encode('utf-8'), addr)
                
                # Log corrections periodically
                if packet_counter % 500 == 0:
                    corr_str = ", ".join([f"{axis}={corrections[axis]:.4f}" for axis in corrections])
                    print(f"Applied corrections: {corr_str}")
                    logging.info(f"Applied corrections: {corr_str}")
                
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
    
    print("KUKA RSI Position Controller")
    print("---------------------------")
    print("This program allows controlling robot position using RSI.")
    
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