import rclpy 
from rclpy.node import Node

import cv2.aruco

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Int32, Bool
from std_srvs.srv import SetBool

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
        self.marker_detection_active = True


        self.aruco_docking_sub = self.create_subscription(
            Bool,
            'blueye/docking_init',
            self.docking_callback,
            10)

        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10)
        
        self.publisher = self.create_publisher(
            Image,
            'detected_markers_image',
            10)

        self.ar_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'blueye/aruco_pose_board',
            10)

        self.ar_pose_single_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'blueye/aruco_pose_single',
            10)

        self.marker_id_pub = self.create_publisher(
            Int32,
            'blueye/marker_id',
            10)

        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        #self.camera_matrix = np.array([[962.54, 0, 960], [0, 969.87, 540], [0, 0, 1]])
        #self.dist_coeffs = np.array([-0.1955, 0.1369, 0.0025, 0.0011, 0.0866])
        self.camera_matrix = np.array([[345.178130, 0.000000, 959.594649],[0.000000, 345.187259, 539.495259],[0.000000, 0.000000, 1.000000]])
        self.dist_coeffs = np.array([0.000022, -0.000003, 0.000003, 0.000003, 0.000000])

        self.board = cv2.aruco.Board(pos_board, self.dictionary, id_board)

        # Predefined rotation matrices for tags
        self.rot_matrix_90_x = self.create_rotation_matrix('x', 90)
        self.rot_matrix_180_y = self.create_rotation_matrix('y', 180)
        self.rot_matrix_neg90_y = self.create_rotation_matrix('y', -90)
        self.rot_matrix_90_y = self.create_rotation_matrix('y', 90)
        self.rot_matrix_neg90_y = self.create_rotation_matrix('y', -90)
        self.rot_matrix_180_x = self.create_rotation_matrix('x', 180)
        self.rot_matrix_neg90_z = self.create_rotation_matrix('z', -90)

        self.rot_matrix_identity = np.eye(3)
        self.docking_init = False

        self.marker_detection_service = self.create_service(SetBool, '/sensor/set_marker_detection', self.handle_set_marker_detection)

    def handle_set_marker_detection(self, request, response):
        self.marker_detection_active = request.data
        response.success = True
        response.message = "Marker detection is now " + ("active" if self.marker_detection_active else "inactive")
        return response
    
    def rotation_matrix_to_quaternion(self, R):
        """
        Convert a rotation matrix to a quaternion.
        """
        q = np.empty((4, ))
        t = np.trace(R)
        if t > 0.0:
            t = np.sqrt(t + 1.0)
            q[0] = 0.5 * t
            t = 0.5 / t
            q[1] = (R[2, 1] - R[1, 2]) * t
            q[2] = (R[0, 2] - R[2, 0]) * t
            q[3] = (R[1, 0] - R[0, 1]) * t
        else:
            i = 0
            if R[1, 1] > R[0, 0]:
                i = 1
            if R[2, 2] > R[i, i]:
                i = 2
            j = (i + 1) % 3
            k = (j + 1) % 3
            t = np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0)
            q[i + 1] = 0.5 * t
            t = 0.5 / t
            q[0] = (R[k, j] - R[j, k]) * t
            q[j + 1] = (R[j, i] + R[i, j]) * t
            q[k + 1] = (R[k, i] + R[i, k]) * t
        return q

    def create_rotation_matrix(self, axis, angle):
        """
        Create a rotation matrix for a given axis ('x', 'y', 'z') and angle in degrees.
        """
        angle = np.radians(angle)  # Convert angle from degrees to radians
        if axis == 'x':
            R = np.array([[1, 0, 0],
                          [0, np.cos(angle), -np.sin(angle)],
                          [0, np.sin(angle), np.cos(angle)]])
        elif axis == 'y':
            R = np.array([[np.cos(angle), 0, np.sin(angle)],
                          [0, 1, 0],
                          [-np.sin(angle), 0, np.cos(angle)]])
        elif axis == 'z':
            R = np.array([[np.cos(angle), -np.sin(angle), 0],
                          [np.sin(angle), np.cos(angle), 0],
                          [0, 0, 1]])
        else:
            raise ValueError("Axis must be 'x', 'y', or 'z'")
        return R

    def image_callback(self, msg: Image):
        if not self.marker_detection_active:
            return


        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.docking_init:
            output_image = self.aruco_pose_msg(image)
            cv2.imshow('Detected single markers', output_image)
        else:
            output_image = self.aruco_pose_single(image)
            cv2.imshow('Detected markers', output_image)
        self.publisher.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))

    @staticmethod
    def aruco_display(corners, ids, rejected, image, rvec, tvec):
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return image

    def aruco_pose_msg(self, frame):
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
            
            if marker_detected:
                object_points = []
                image_points = []

                for i, marker_id in enumerate(markerIds.flatten()):
                    idx = np.where(id_board == marker_id)[0][0]
                    object_points.extend(pos_board[idx])
                    image_points.extend(markerCorners[i].reshape(-1, 2))

                object_points = np.array(object_points, dtype=np.float32)
                image_points = np.array(image_points, dtype=np.float32)

                reprojected_points, jacobian = cv2.projectPoints(object_points, rvec_board, tvec_board, self.camera_matrix, self.dist_coeffs)
                error = image_points - reprojected_points.reshape(-1, 2)
                mean_error = np.mean(np.linalg.norm(error, axis=1))

                covariance = np.eye(6) * mean_error


                T_body_camera = np.zeros((4, 4), dtype=float)
                T_body_camera[:3, :3] = self.rot_matrix_identity
                T_body_camera[:3, 3] = np.array([0.21, 0.0, 0.09])
                T_body_camera[3, 3] = 1
                
                T_tag_camera = np.zeros((4, 4), dtype=float)
                T_tag_camera[:3, :3] = np.matrix(cv2.Rodrigues(rvec_board)[0])
                T_tag_camera[:3, 3] = tvec_board.flatten()
                T_tag_camera[3, 3] = 1

                T_camera_tag = np.linalg.inv(T_tag_camera)

                T_tag_dock = np.zeros((4, 4), dtype=float)
                T_tag_dock[:3, :3] = self.rot_matrix_180_y
                T_tag_dock[:3, 3] =  np.array([1.4925, 1.919, 1.175])
                T_tag_dock[3, 3] = 1

                T_dock_world = np.zeros((4, 4), dtype=float)
                T_dock_world[:3, :3] = self.rot_matrix_90_x
                T_dock_world[:3, 3] = np.array([-1.0, 1.7, 0])
                T_dock_world[3, 3] = 1

                T_camera_world = T_dock_world @ T_tag_dock @ T_camera_tag @ T_body_camera

                R_world_blueye_board = T_camera_world[:3, :3]
                t_world_blueye_board = T_camera_world[:3, 3]

                pose_blueye_ned_board = np.array([t_world_blueye_board[0], t_world_blueye_board[1], t_world_blueye_board[2]])
                quaternion_blueye_ned_board = self.rotation_matrix_to_quaternion(R_world_blueye_board)

                cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec_board, tvec_board, 0.25)

                frame = self.aruco_display(markerCorners, markerIds, rejectedCanditates, frame, rvec_board, tvec_board) 

                '''Adding pose estimation to the image
                Since robot_localization needs PoseWithCovarianceStamped message, 
                we need to publish the pose estimation as a PoseWithCovarianceStamped message'''
                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header.frame_id = "odom"
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.pose.pose.position.x = pose_blueye_ned_board[0]
                pose_msg.pose.pose.position.y = pose_blueye_ned_board[1]
                pose_msg.pose.pose.position.z = pose_blueye_ned_board[2]
                pose_msg.pose.pose.orientation.x = quaternion_blueye_ned_board[0]
                pose_msg.pose.pose.orientation.y = quaternion_blueye_ned_board[1]
                pose_msg.pose.pose.orientation.z = quaternion_blueye_ned_board[2]
                pose_msg.pose.pose.orientation.w = quaternion_blueye_ned_board[3]
                pose_msg.pose.covariance = covariance.flatten().tolist()

                self.ar_pose_pub.publish(pose_msg)

        return frame

    def aruco_pose_single(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        markerCorners, markerIds, rejectedCanditates = self.detector.detectMarkers(
            gray,
            self.camera_matrix,
            self.dist_coeffs)

        if len(markerCorners) > 0:
            prioritized_tags = [1, 16, 17, 18, 19, 20]
            detected_tags = markerIds.flatten()
            rvec_init = np.zeros((3, 1), dtype=np.float32)
            tvec_init = np.zeros((3, 1), dtype=np.float32)

            tag_pose = {
                1: np.array([1.4925, 1.919, 1.175]),
                16: np.array([2.02, 1.87, 1.7]),
                17: np.array([0.56, 1.88, 1.18]),
                18: np.array([0.0, 1.87, 1.7]),
                19: np.array([0.23, 1.87, 2.18]),
                20: np.array([1.00, 1.87, 2.18])
            }

            tag_rotation = {
            1: self.rot_matrix_180_y,
            16: self.rot_matrix_90_y,
            17: self.rot_matrix_180_y,
            18: self.rot_matrix_neg90_y,
            19: self.rot_matrix_identity,
            20: self.rot_matrix_identity
            }

            marker_length = {
                1: 0.15,
                16: 0.222,
                17: 0.222,
                18: 0.222,
                19: 0.222,
                20: 0.222
            }


            for tag in prioritized_tags:
                if tag in detected_tags:
                    tag_index = np.where(detected_tags == tag)[0]
                    index = tag_index[0]
                    corner = [markerCorners[index]]
                    marker_length_tag = marker_length[tag]

                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(
                        corner, marker_length_tag, self.camera_matrix, self.dist_coeffs, rvec_init, tvec_init)
                    rvec_single = rvec
                    tvec_single = tvec

                    object_points = np.array([[-marker_length_tag/2, marker_length_tag/2, 0],
                            [marker_length_tag/2, marker_length_tag/2, 0],
                            [marker_length_tag/2, -marker_length_tag/2, 0],
                            [-marker_length_tag/2, -marker_length_tag/2, 0]], dtype=np.float32)
                    image_points = np.array(corner[0], dtype=np.float32)

                    # Reproject points to calculate the reprojection error
                    reprojected_points, jacobian = cv2.projectPoints(object_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)
                    error = image_points - reprojected_points.reshape(-1, 2)
                    mean_error = np.mean(np.linalg.norm(error, axis=1))
                    print("Jacobian", jacobian)
                    jacobian_flat = jacobian.reshape(-1, jacobian.shape[-1])

                    # Calculate the covariance matrix
                    H = jacobian_flat.T @ jacobian_flat  # Compute the product of the transpose and the Jacobian
                    covariance_matrix_full = np.linalg.pinv(H) * mean_error 

                    # Assuming the first 3x3 block corresponds to rotation and the next 3x3 block to translation
                    covariance_rotation = covariance_matrix_full[:3, :3]
                    covariance_translation = covariance_matrix_full[3:6, 3:6]

                    # Construct a 6x6 covariance matrix
                    covariance_6x6 = np.zeros((6, 6))
                    covariance_6x6[:3, :3] = covariance_translation
                    covariance_6x6[3:6, 3:6] = covariance_rotation

                    covariance = covariance_6x6



                    T_body_camera = np.zeros((4, 4), dtype=float)
                    T_body_camera[:3, :3] = self.rot_matrix_identity
                    T_body_camera[:3, 3] = np.array([0.21, 0.0, 0.09])
                    T_body_camera[3, 3] = 1    


                    T_tag_cam = np.zeros((4, 4), dtype=float)
                    T_tag_cam[:3, :3] = np.matrix(cv2.Rodrigues(rvec_single)[0])
                    T_tag_cam[:3, 3] = tvec_single
                    T_tag_cam[3, 3] = 1

                    T_cam_tag = np.linalg.inv(T_tag_cam)

                    T_tag_dock = np.zeros((4, 4), dtype=float)
                    T_tag_dock[:3, :3] = tag_rotation[tag]
                    T_tag_dock[:3, 3] = tag_pose[tag]
                    T_tag_dock[3, 3] = 1

                    T_dock_world = np.zeros((4, 4), dtype=float)
                    T_dock_world[:3, :3] = self.rot_matrix_90_x
                    T_dock_world[:3, 3] = np.array([-1.0, 1.7, 0])
                    T_dock_world[3, 3] = 1

                    

                    T_cam_world = T_dock_world @ T_tag_dock @ T_cam_tag @ T_body_camera
                    t_station_blueye_tag = T_cam_world[:3, 3]

                    pose_single_marker = np.array([t_station_blueye_tag[0], t_station_blueye_tag[1], t_station_blueye_tag[2]])

                    cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec_single, tvec_single, 0.1)
                    frame = self.aruco_display(markerCorners, markerIds, rejectedCanditates, frame, rvec_single, tvec_single)

                    

                    pose_msg = PoseWithCovarianceStamped()
                    pose_msg.header.frame_id = "odom"
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.pose.pose.position.x = float(pose_single_marker[0])
                    pose_msg.pose.pose.position.y = float(pose_single_marker[1])
                    pose_msg.pose.pose.position.z = float(pose_single_marker[2])
                    pose_msg.pose.covariance = covariance.flatten().tolist()
                    self.ar_pose_single_pub.publish(pose_msg)

                    marker_id_msg = Int32()
                    marker_id_msg.data = tag
                    self.marker_id_pub.publish(marker_id_msg)
                    break

        return frame
    
    def docking_callback(self, msg: Bool):
        self.docking_init = msg.data


def main():
    rclpy.init()
    blueye_img = blueye_image()
    rclpy.spin(blueye_img)
    blueye_img.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
