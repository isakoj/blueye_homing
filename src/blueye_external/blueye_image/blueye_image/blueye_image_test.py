import rclpy 
from rclpy.node import Node

import cv2.aruco

from geometry_msgs.msg import PoseWithCovarianceStamped

import transforms3d as tf3d

import cv2

import numpy as np
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from blueye_image.aruco_board_final import pos_board, id_board


class blueye_image(Node):
    ARUCO_DICT = {
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    }

    def __init__(self):
        super().__init__('blueye_image')
        self.subscription = self.create_subscription(
            Image,
            'blueye/front_camera',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(
            Image,
            'detected_markers_image',
            10)
        
        self.ar_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'blueye/aruco_pose',
            10)
        
        self.tag_id_pub = self.create_publisher(
        )
        

        self.bridge = CvBridge()
        # self.aruco_type = "DICT_5X5_50"
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        
        self.camera_matrix = np.array([[962.54, 0, 960], [0, 969.87, 540], [0, 0, 1]])
        self.dist_coeffs = np.array([-0.1955, 0.1369, 0.0025, 0.0011, 0.0866])

        self.board = cv2.aruco.Board(pos_board, self.dictionary, id_board)


    # Callback function that is executed when we receive an image from the gazebo topic blueye/front_camera   
    def image_callback(self, msg:Image):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        output_image = self.aruco_marker_msg(image)
        
        cv2.imshow('Detected markers', output_image)

        self.publisher.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))

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
                
        return image



    def aruco_marker_msg(self, frame):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        markerCorners, markerIds, rejectedCanditates = self.detector.detectMarkers(
                                                                        gray, 
                                                                        self.camera_matrix, 
                                                                        self.dist_coeffs)

        if len(markerCorners) > 0:

            rvec_init = np.zeros((3, 1), dtype=np.float32)
            tvec_init = np.zeros((3, 1), dtype=np.float32)

            marker_detected, rvec_board, tvec_board = cv2.aruco.estimatePoseBoard(
                markerCorners, markerIds, self.board, self.camera_matrix, self.dist_coeffs, rvec_init, tvec_init)

            cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec_board, tvec_board, 0.1)

            frame = self.aruco_display(markerCorners, markerIds, rejectedCanditates, frame, rvec_board, tvec_board) 
        return frame

def main():
    rclpy.init()
    blueye_img = blueye_image()
    rclpy.spin(blueye_img)
    blueye_img.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
