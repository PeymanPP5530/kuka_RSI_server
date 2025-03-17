import socket
import xml.etree.ElementTree as ET

def create_response(ipoc):
    """Generate XML response exactly matching the screenshot format, without whitespace."""
    response = '<Sen Type="ImFree"><EStr>Message from RSI TestServer</EStr><Tech T21="1.09" T22="2.08" T23="3.07" T24="4.06" T25="5.05" T26="6.04" T27="7.03" T28="8.02" T29="9.01" T210="10.00" /><RKorr X="0.0256" Y="0.0256" Z="0.0256" A="0.0000" B="0.0000" C="0.0000" /><DiO>125</DiO><IPOC>' + ipoc + '</IPOC></Sen>'
    return response

def main():
    HOST, PORT = "10.10.10.20", 59152
    
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
                
                # Extract RIst attributes
                rist_elem = root.find(".//RIst")
                if rist_elem is not None:
                    x = float(rist_elem.get("X", 0.0))
                    y = float(rist_elem.get("Y", 0.0))
                    z = float(rist_elem.get("Z", 0.0))
                    a = float(rist_elem.get("A", 0.0))
                    b = float(rist_elem.get("B", 0.0))
                    c = float(rist_elem.get("C", 0.0))
                    print(f"RIst Values1111111: X={x}, Y={y}, Z={z}, A={a}, B={b}, C={c}")
                else:
                    print("Warning: No RIst found in received XML")
            except ET.ParseError as e:
                print(f"Error parsing XML: {e}")
                continue
            
    
            response_xml = create_response(ipoc)
            sock.sendto(response_xml.encode('utf-8'), addr)
            print(f"Sent response:")
            print(response_xml)
            
        except Exception as e:
            print(f"Error in communication: {e}")
            
if __name__ == "__main__":
    main()