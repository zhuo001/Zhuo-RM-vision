import socket
import time

UDP_IP = "192.168.1.2"
UDP_PORT = 6201

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Listening on {UDP_IP}:{UDP_PORT}...")
    sock.settimeout(5.0)
    
    count = 0
    start_time = time.time()
    while count < 10:
        data, addr = sock.recvfrom(1024)
        print(f"Received packet from {addr}, length: {len(data)}")
        count += 1
        
    print("UDP connection verified!")
except socket.timeout:
    print("No data received (Timeout)")
except Exception as e:
    print(f"Error: {e}")
finally:
    sock.close()
