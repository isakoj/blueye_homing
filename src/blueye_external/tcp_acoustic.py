import socket
import math
import json
from nav_msgs.msg import Odometry
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Quaternion, Pose, Point, TwistWithCovarianceStamped, Vector3
import numpy as np
import time

# Script that fetches the velocities, vx, vy, vz, from the Waterlinked DVL attached to the Blueye in guestport
# Utilizing sockets to communicate with the DVL

def tcp_client(server_ip, server_port):
    # Create a socket object using the socket module
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    # Connect to the server
    sock.connect((server_ip, server_port))
    print(f"Connected to {server_ip} on port {server_port}")


    # Start ros
    rclpy.init()
    odometry_node = Node("blueye_odom")
    dvl_pos_pub = odometry_node.create_publisher(Odometry, "/blueye/odometry", 1)
    dvl_vel_pub = odometry_node.create_publisher(TwistWithCovarianceStamped, "/blueye/dvl_enu", 1)
    
    try:
        # Keep receiving data from the server and print it
        while True:
            data = sock.recv(2048)  # Buffer size is 1024 bytes
            if not data:
                break  # Exit the loop if no data is received
            try:
                decoded_data = data.decode()
                decoded_data = decoded_data.strip('"')
                # Convert string to JSON (dictionary)
                json_data = json.loads(decoded_data)

                #print(json_data)

                if json_data["type"] == "velocity":
                    print("Velocity")
                    print("Vx: {}, Vy: {}, Vz: {}".format(json_data["vx"], json_data["vy"], json_data["vz"]))
                    print(json_data)
                    print("--------------------------------------------------------------------")
                    
                    dvl_stamped = TwistWithCovarianceStamped()
        
                    dvl_stamped.header.stamp = odometry_node.get_clock().now().to_msg() # WILL THIS WORK??
                    dvl_stamped.header.frame_id = "base_link" # Correct???
        
                    dvl_stamped.twist.twist.linear.x = float(json_data["vx"])
                    dvl_stamped.twist.twist.linear.y = float(-json_data["vy"]) # Negative because converting from NED to ENU
                    dvl_stamped.twist.twist.linear.z = float(-json_data["vz"]) # Negative because converting from NED to ENU
                    
                    
                    # Adding covariance matrix (R) for the measurements
                    
                    # Assuming example standard deviations (std_dev_x, std_dev_y, std_dev_z) for each velocity component
                    std_dev_vx = 0.01  # Example standard deviation for vx
                    std_dev_vy = 0.01  # Example standard deviation for vy
                    std_dev_vz = 0.01  # Example standard deviation for vz
                    covariance = np.diag([std_dev_vx**2, std_dev_vy**2, std_dev_vz**2, 0, 0, 0]) # Only setting the diagonal for linear velocities

                    dvl_stamped.twist.covariance = covariance.flatten().tolist()        

                    dvl_vel_pub.publish(dvl_stamped)
                    
                    # Eventuelt hent ut covariance fra json_data?? Spør Ambjørn om dette                    
                    # if "covariance" in json_data:
                    #     # Assuming the covariance array is provided in row-major order for a 3x3 matrix
                    #     cov = json_data["covariance"]
                    #     # Fill the covariance matrix for TwistWithCovarianceStamped
                    #     msg.twist.covariance = [
                    #         cov[0], cov[1], cov[2], 0, 0, 0,
                    #         cov[3], cov[4], cov[5], 0, 0, 0,
                    #         cov[6], cov[7], cov[8], 0, 0, 0,
                    #         0, 0, 0, 0, 0, 0,
                    #         0, 0, 0, 0, 0, 0,
                    #         0, 0, 0, 0, 0, 0
                    #     ]
                    
                    
                    
                elif json_data["type"] == "position_local":
                    print("Position")
                    print("X: {}, Y: {}, Z {}".format(json_data["x"], json_data["y"], json_data["z"]))
                    print("Roll: {}, Pitch: {}, Yaw: {}".format(json_data["roll"], json_data["pitch"], json_data["yaw"]))
                    print("--------------------------------------------------------------------")
                    
                    msg = Odometry() 
                    msg.pose.pose.position.x = float(json_data["x"])
                    msg.pose.pose.position.y = float(json_data["y"])
                    msg.pose.pose.position.z = float(json_data["z"])

                    yaw = (json_data["yaw"]*math.pi)/180.0
                    msg.pose.pose.orientation.w = yaw
                    dvl_pos_pub.publish(msg)

                    #print(json_data)
                else:
                    print(json_data)


            except json.JSONDecodeError:
                pass
                #print("Received non-JSON data:", decoded_data)

    finally:
        # Close the connection
        sock.close()
        print("Connection closed.")

# Example usage
if __name__ == "__main__":
    HOST = '192.168.'  # Server IP address
    PORT = 16171            # Server port number
    
    tcp_client(HOST, PORT)