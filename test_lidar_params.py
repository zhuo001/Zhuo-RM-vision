import subprocess
import time
import socket
import os
import signal

def check_udp_packet_size(timeout=5):
    UDP_IP = "192.168.1.2"
    UDP_PORT = 6201
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(timeout)
    max_size = 0
    start_time = time.time()
    try:
        while time.time() - start_time < timeout:
            data, _ = sock.recvfrom(2048)
            if len(data) > max_size:
                max_size = len(data)
            if max_size > 100: # Found large packet
                break
    except socket.timeout:
        pass
    finally:
        sock.close()
    return max_size

def test_params(init_type, work_mode):
    print(f"Testing initialize_type={init_type}, work_mode={work_mode}...")
    
    # Construct command
    cmd = [
        "ros2", "run", "unitree_lidar_ros2", "unitree_lidar_ros2_node",
        "--ros-args",
        "-p", f"initialize_type:={init_type}",
        "-p", f"work_mode:={work_mode}",
        "-p", "lidar_ip:=192.168.1.1",
        "-p", "local_ip:=192.168.1.2",
        "-p", "lidar_port:=6101",
        "-p", "local_port:=6201"
    ]
    
    # Start ROS node
    process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    # Wait for initialization
    time.sleep(3)
    
    # Check UDP
    max_size = check_udp_packet_size(timeout=3)
    print(f"  -> Max packet size: {max_size} bytes")
    
    if max_size > 100:
        print("  -> SUCCESS! Point cloud data detected.")
    else:
        print("  -> No point cloud data.")
        
    # Cleanup
    process.send_signal(signal.SIGINT)
    process.wait()
    time.sleep(1)

# Main test loop
print("Starting parameter sweep...")
# Ensure environment is sourced (this script should be run in an environment where ros2 is available)
# We will assume the user runs this with the correct environment.

# Test cases
test_params(1, 0) # Try Type 1
test_params(2, 0) # Try Type 2 (Default)
test_params(1, 1) # Try Type 1, Mode 1
test_params(2, 1) # Try Type 2, Mode 1

print("Test complete.")
