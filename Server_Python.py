import socket
import xml.etree.ElementTree as ET

def create_response(ipoc, korr_x, korr_y, korr_z, korr_a, korr_b, korr_c):
    """Generate XML response including calculated RKorr values."""
    response = f'<Sen Type="ImFree"><EStr>Message from RSI TestServer</EStr>' \
               f'<Tech T21="1.09" T22="2.08" T23="3.07" T24="4.06" T25="5.05" T26="6.04" T27="7.03" T28="8.02" T29="9.01" T210="10.00" />' \
               f'<RKorr X="{korr_x:.4f}" Y="{korr_y:.4f}" Z="{korr_z:.4f}" A="{korr_a:.4f}" B="{korr_b:.4f}" C="{korr_c:.4f}" />' \
               f'<DiO>125</DiO><IPOC>{ipoc}</IPOC></Sen>'
    return response

def main():
    HOST, PORT = "10.10.10.20", 59152
    
    # Get user input for desired X, Y, Z, A, B, C
    print("Enter the desired position and orientation values:")
    target_x = float(input("Target X: "))
    target_y = float(input("Target Y: "))
    target_z = float(input("Target Z: "))
    target_a = float(input("Target A: "))
    target_b = float(input("Target B: "))
    target_c = float(input("Target C: "))

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

                # Extract IPOC
                ipoc_elem = root.find(".//IPOC")
                ipoc = ipoc_elem.text if ipoc_elem is not None else "0000000"

                # Extract RIst attributes
                rist_elem = root.find(".//RIst")
                if rist_elem is not None:
                    x = float(rist_elem.get("X", 0.0))
                    y = float(rist_elem.get("Y", 0.0))
                    z = float(rist_elem.get("Z", 0.0))
                    a = float(rist_elem.get("A", 0.0))
                    b = float(rist_elem.get("B", 0.0))
                    c = float(rist_elem.get("C", 0.0))
                    print(f"RIst Values: X={x}, Y={y}, Z={z}, A={a}, B={b}, C={c}")
                else:
                    print("Warning: No RIst found in received XML")
                    continue

                # Calculate corrections
                korr_x = 0.0255 if abs(target_x - x) > 1 else 0
                korr_y = 0.0255 if abs(target_y - y) > 1 else 0
                korr_z = 0.0255 if abs(target_z - z) > 1 else 0
                korr_a = 0.0255 if abs(target_a - a) > 1 else 0
                korr_b = 0.0255 if abs(target_b - b) > 1 else 0
                korr_c = 0.0255 if abs(target_c - c) > 1 else 0
                print(f"Corrections: X={korr_x:.4f}, Y={korr_y:.4f}, Z={korr_z:.4f}, A={korr_a:.4f}, B={korr_b:.4f}, C={korr_c:.4f}")

            except ET.ParseError as e:
                print(f"Error parsing XML: {e}")
                continue

            # Create response and send it back
            response_xml = create_response(ipoc, korr_x, korr_y, korr_z, korr_a, korr_b, korr_c)
            sock.sendto(response_xml.encode('utf-8'), addr)
            print(f"Sent response:")
            print(response_xml)
            
        except Exception as e:
            print(f"Error in communication: {e}")

if __name__ == "__main__":
    main()
