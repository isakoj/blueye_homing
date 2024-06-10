import rclpy 
from rclpy.node import Node
import sys, time, math
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from scipy.spatial.transform import Rotation
import time

#Importing the aruco board definition
from blueye_image.aruco_board_final import pos_board, id_board

# Defining Image node
class blueye_image(Node):
    ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
    }

    # Specify which type of marker you want to detect
    aruco_type = "DICT_5X5_50"
    
    # Here you define your Camera matrix (K-matrix) and the distortion coefficients of your camera
    # These values are obtained from camera calibration (Here I use the ones listed in gz topic -e -t /blueye/camera_info)
    intrinsic_camera = np.array(((345.86108207702637, 0, 960),(0,345.86108922958374, 540),(0,0,1)))
    distortion = np.array((0.0,0.0,0.0,0.0,0.0))
    
    """ (Here is the values from Johannes at Blueye)
    intrinsic_camera = np.array(((962.54, 0, 960),(0,969.87, 540),(0,0,1)))
    distortion = np.array((-0.1955,0.1369,0.0025,0.0011,0.0866)) 
    """
    
    # Tag pos (midpoint) wrt dock frame expressed in dock frame
    # Find these from Blender     
    t_dock_tag_list = [ [1.975, 1.46, 1.176],
                        [1.015, 1.46, 1.176],
                        [1.495, 1.89, 1.176],
                        [1.495, 1.61, 2.456]]
    
    # Rotation matrix +180 deg around the y-axis
    rot_matrix_180_y = np.array([
        [-1, 0, 0],
        [0, 1, 0],
        [0, 0, -1]
    ])

    # Rotation matrix +90 deg around the x-axis
    rot_matrix_90_x = np.array([
        [1, 0, 0],
        [0, 0, -1],
        [0, 1, 0]
    ])
    
    # 180 deg rotation matrix around the x axis
    rot_matrix_180_x  = np.zeros((3,3), dtype=np.float32)
    rot_matrix_180_x[0,0] = 1.0
    rot_matrix_180_x[1,1] =-1.0
    rot_matrix_180_x[2,2] =-1.0
    
    #-- Font for the text in the image
    font = cv2.FONT_HERSHEY_PLAIN
    
    # Aruco Board present or not
    board = True
    
    # Statistical Counters for tag performance test
    detection_time = []
    tot_images = 0
    tot_detections = 0 
    tot_correct_detections = 0
    TP = 0.0
    FP = 0.0
    FN = 0.0
    id_precision = 0.0
    detection_rate = 0.0
    avg_detection_time = 0
    
    def __init__(self):
        super().__init__("blueye_image")
        # Create subscribers
        self.image_subscriber = self.create_subscription(
                Image, "blueye/front_camera", self.image_callback, 1)
        self.pose_subscriber = self.create_subscription(
                PoseArray, "model/blueye/pose", self.pose_callback, 1)
        # Create publishers

        self.delta_error_control_pub_ = self.create_publisher(Pose, "/blueye/delta_error_control", 10)
        self.delta_error_actual_pub_ = self.create_publisher(Pose, "/blueye/delta_error_actual", 10)
        self.pose_euler_gt_pub_ = self.create_publisher(Pose, "/blueye/pose_euler_gt", 10)
        self.pose_estimated_pub_ = self.create_publisher(Pose, "/blueye/pose_estimated", 10)
        self.pose_estimated_board_pub_ = self.create_publisher(Pose, "/blueye/pose_estimated_board", 10)
        
        self.bridge = CvBridge()
        
        self.timer = self.create_timer(1,self.write_statistical_values_to_file)
    
 
    def write_statistical_values_to_file(self):
        statistical_values = f"True Positives: {self.TP}, False Positives: {self.FP}, False Negatives: {self.FN}, ID Precision: {self.id_precision}, Detection Rate: {self.detection_rate}, Avg. Detection Time: {self.avg_detection_time},\n"
        with open("statistical_values.txt", "a") as file:
            file.write(statistical_values)

            
    # Callback function that is executed when we receive an image from the gazebo topic blueye/front_camera   
    def image_callback(self, msg:Image):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        output_image = self.pose_estimation(image, self.ARUCO_DICT[self.aruco_type], self.intrinsic_camera, self.distortion)
        
        cv2.imshow('Estimated Pose Image', output_image)
        key = cv2.waitKey(1) & 0xFF
        """ 
        if key == ord('q'):
                break 
        """
        # Statistical parameters for tag performance test
        self.tot_images +=1
        self.TP = self.tot_correct_detections # True positives, because tag is present in every image
        self.FP = self.tot_detections - self.tot_correct_detections # False positives
        self.FN = self.tot_images - self.tot_correct_detections # False negatives
        if self.tot_detections != 0: 
            self.id_precision = self.TP/self.tot_detections
        else:
            self.id_precision = None
        if self.tot_images != 0:
            self.detection_rate = self.TP/self.tot_images
        else:
            self.detection_rate = None
    
    
    # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6


    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ). So here the order is xyz??
    def rotationMatrixToEulerAngles(self, R):
        assert (self.isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])
    
    # Label the tags with ID
    @staticmethod
    def aruco_display(corners, ids, rejected, image, rvec, tvec):
        if len(corners) > 0:            
            
            # Transforms a multidimensional array into a 1d-array
            ids = ids.flatten()
            # Iterates over two sets of data simultaneously, 
            # automatically creating matching tuples with 4 corners and 1 Id
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # Takes out the corner coords casted to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                # Label the Aruco with ID
                cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                #-- Print the tag position in camera frame
                #str_position = "Marker Position x =%4.2f  y =%4.2f  z =%4.2f"%(tvec[0], tvec[1], tvec[2])
                #cv2.putText(image, str_position, (topLeft[0], topLeft[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
                
                #print("[Inference] ArUco marker ID: {}".format(markerID))
        return image
    
    # This function is executed for every image received    
    def pose_estimation(self, frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
        
        # frame = image
        # Converting picture from BGR to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
        parameters = cv2.aruco.DetectorParameters_create()
        aruco_board = cv2.aruco.Board_create(pos_board, cv2.aruco_dict, id_board)
        
        # Perform this line to activate the cornerSubPix function that refines the corner locations
        # parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
            cameraMatrix=matrix_coefficients,
            distCoeff=distortion_coefficients)

        # Initialize a pose array of zeros
        pose_blueye_wrt_world_board = np.zeros(4)
        
        
        # Check that we actually are detecting any tags
        if len(corners) > 0:
            
            # Test denne!!! If new markers are detected in this step they are automatically added to the lists ids and corners?
            # OBS: Require 2D board
            # aruco.refineDetectedMarkers(gray, aruco_board, corners, ids, rejected_img_points)
            
            self.tot_detections +=len(ids)
 
            # -------Aruco Board Pose Estimation----------------------------------------
        
            # Defines initial guesses
            rvec_initial_guess = np.zeros((3, 1), dtype=np.float32)
            tvec_initial_guess = np.zeros((3, 1), dtype=np.float32)
            
            # Estimating board pose
            # OBS This pose est is going to replace the singlemarker est above!!!
            marker_detected, rvec_board, tvec_board = cv2.aruco.estimatePoseBoard(corners, ids, aruco_board, matrix_coefficients, distortion_coefficients, rvec_initial_guess, tvec_initial_guess)
            
            # Obtain the rotation matrix tag->camera -- Board
            R_ct_board    = np.matrix(cv2.Rodrigues(rvec_board)[0]) 
            R_tc_board    = R_ct_board.T  
            
            # Get the attitude of camera frame wrt flipped tag frame(or vice verca?????), in terms of euler 321 (Needs to be flipped first)    
            roll_camera_board, pitch_camera_board, yaw_camera_board = self.rotationMatrixToEulerAngles(self.rot_matrix_180_x*R_tc_board)
            
            # Pitch_camera is the yaw angle because the y-axis of the aruco frame is upwards
            # yaw_blueye is wrt world frame. Minus pi/2 is because the aruco tags are rotated pi/2 relative to world frame
            # The minus is because our flipped tag frame has its y-axis(ref pitch) pointing opposite of world frames z-axis
            yaw_blueye_board = -pitch_camera_board - math.pi/2
        
            #R^world_dock
            R_world_dock =  self.rot_matrix_90_x
            #t^world_dock: position of dock frame wrt world frame expressed in world frame
            t_world_dock =  np.array([0,0,-97.3])
            #T^world_dock 
            T_world_dock =  np.zeros((4,4), dtype=float)
            T_world_dock[:3,:3] = R_world_dock
            T_world_dock[:3,3] = t_world_dock
            T_world_dock[3,3] = 1
            
            R_dock_tag = self.rot_matrix_180_y
            t_dock_tag = [1.4925, 1.9199, 1.176] # Origo of tag board is in center of tag 1
            T_dock_tag =  np.zeros((4,4), dtype=float)
            T_dock_tag[:3,:3] = R_dock_tag
            T_dock_tag[:3,3] = t_dock_tag
            T_dock_tag[3,3] = 1
            
            R_tag_camera = R_tc_board
            t_tag_camera = (-R_tc_board @ np.array(tvec_board).reshape(-1,1)).flatten()
            T_tag_camera =  np.zeros((4,4), dtype=float)
            T_tag_camera[:3,:3] = R_tag_camera
            T_tag_camera[:3,3] = t_tag_camera
            T_tag_camera[3,3] = 1
            
            R_camera_blueye = np.eye(3)
            t_camera_blueye = np.array([0.0015,0.09,-0.209])
            T_camera_blueye =  np.zeros((4,4), dtype=float)
            T_camera_blueye[:3,:3] = R_camera_blueye
            T_camera_blueye[:3,3] = t_camera_blueye
            T_camera_blueye[3,3] = 1
            
            # Coordinate frames transformation
            T_world_blueye_board = T_world_dock @ T_dock_tag @ T_tag_camera @ T_camera_blueye
            
            R_world_blueye_board = T_world_blueye_board[:3,:3]
            t_world_blueye_board = T_world_blueye_board[:3,3] # Merk: denne er den samme som pos_blueye_wrt_world 
            
            pose_blueye_wrt_world_board = np.array([t_world_blueye_board[0], t_world_blueye_board[1], t_world_blueye_board[2], yaw_blueye_board], dtype=float)
            
            # Visualize the detected tags and their coordinate frame
            cv2.aruco.drawDetectedMarkers(frame, corners) 
            cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec_board, tvec_board, length=0.25) 
            
            # Adding Id-label
            frame = self.aruco_display(corners, ids, rejected_img_points, frame, rvec_board, tvec_board)
            

            # Desired end position and yaw angle when fully docked expressed in world frame
            x_d = 1.507
            y_d = -1.978
            z_d = -95.706
            yaw_d = -math.pi/2

                    
                
        return frame
    
def main():
    rclpy.init()
    blueye_img = blueye_image()
    rclpy.spin(blueye_img)
    blueye_img.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
