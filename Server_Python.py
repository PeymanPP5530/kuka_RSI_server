import socket
import xml.etree.ElementTree as ET
import numpy as np 

def get_user_input():
    
    x = float(input("Enter X correction: "))
    y = float(input("Enter Y correction: "))
    z = float(input("Enter Z correction: "))
    a = float(input("Enter A correction: "))
    b = float(input("Enter B correction: "))
    c = float(input("Enter C correction: "))
    return x, y, z, a, b, c

def shortest_angle_difference(target, current):
    # Calculate the raw difference
    diff = target - current
    
    # Normalize the difference to be within the range -180 to 180
    if diff > 180:
        diff -= 360
    elif diff < -180:
        diff += 360
        
    return diff

def create_response(ipoc, corr_x, corr_y, corr_z, corr_a, corr_b, corr_c):
    """Generate XML response with calculated corrections."""
    response = (
        f'<Sen Type="ImFree">'
        f'<EStr>Message from RSI TestServer</EStr>'
        f'<Tech T21="1.09" T22="2.08" T23="3.07" T24="4.06" T25="5.05" T26="6.04" T27="7.03" T28="8.02" T29="9.01" T210="10.00" />'
        f'<RKorr X="{corr_x:.4f}" Y="{corr_y:.4f}" Z="{corr_z:.4f}" A="{corr_a:.4f}" B="{corr_b:.4f}" C="{corr_c:.4f}" />'
        f'<DiO>125</DiO>'
        f'<IPOC>{ipoc}</IPOC>'
        f'</Sen>'
    )
    return response

def main():
    HOST, PORT = "10.10.10.20", 59152
    
    target_x, target_y, target_z, target_a, target_b, target_c = get_user_input()
    
    
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    print(f"Server listening on {HOST}:{PORT}")
    
    while True:
        try:
            data, addr = sock.recvfrom(1024)  # Receive data from KUKA
            received_xml = data.decode('utf-8')
            print(f"Received from {addr}:")
            print(received_xml)
            
            
            try:
                root = ET.fromstring(received_xml)
                ipoc_elem = root.find(".//IPOC")
                if ipoc_elem is not None:
                    ipoc = ipoc_elem.text
                else:
                    ipoc = "0000000"
                    print("Warning: No IPOC found in received XML")
                    
                    
                # Extract RIst values using .get()
                rist_elem = root.find(".//RIst")
                if rist_elem is not None:
                    rist_x = float(rist_elem.get("X", "0.0"))  # Default to 0.0 if attribute missing
                    rist_y = float(rist_elem.get("Y", "0.0"))
                    rist_z = float(rist_elem.get("Z", "0.0"))
                    rist_a = float(rist_elem.get("A", "0.0"))
                    rist_b = float(rist_elem.get("B", "0.0"))
                    rist_c = float(rist_elem.get("C", "0.0"))

                    print(f"RIst: X={rist_x}, Y={rist_y}, Z={rist_z}, A={rist_a}, B={rist_b}, C={rist_c}")
                else:
                    print("Warning: No RIst found in received XML")
                    
            except ET.ParseError as e:
                print(f"Error parsing XML: {e}")
                continue
            
            corr_x = 0.0256 if (target_x - rist_x) > 1 else (-0.0256 if (target_x - rist_x) < -1 else 0)
            corr_y = 0.0256 if (target_y - rist_y) > 1 else (-0.0256 if (target_y - rist_y) < -1 else 0)
            corr_z = 0.0256 if (target_z - rist_z) > 1 else (-0.0256 if (target_z - rist_z) < -1 else 0)
            
            diff_a = shortest_angle_difference(target_a, rist_a)
            diff_b = shortest_angle_difference(target_b, rist_b)
            diff_c = shortest_angle_difference(target_c, rist_c)
            
            corr_a = 0.0256 if diff_a > 1 else (-0.0256 if diff_a < -1 else 0)
            corr_b = 0.0256 if diff_b > 1 else (-0.0256 if diff_b < -1 else 0)
            corr_c = 0.0256 if diff_c > 1 else (-0.0256 if diff_c < -1 else 0)

            response_xml = create_response(ipoc,corr_x,corr_y,corr_z,corr_a,corr_b,corr_c)  
            sock.sendto(response_xml.encode('utf-8'), addr)
            print(f"Sent response:")
            print(response_xml)
            
        except Exception as e:
            print(f"Error in communication: {e}")
            
if __name__ == "__main__":
    main()
