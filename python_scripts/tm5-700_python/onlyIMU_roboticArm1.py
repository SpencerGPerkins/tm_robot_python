#!/usr/bin/env python3

import rospy
from math import pi, cos, sin
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from tm_msgs.msg import *
from tm_msgs.srv import *

imu_Rx = 0
imu_Ry = 0

class ListenerArm:
    def __init__(self):
        self.EEpose_position_vector = np.array([.30179, -.1223, .765464])
        self.EEpose_position_previous = np.array([.30179, -.1223, .765464])
        self.Rx = 0
        self.Ry = 80
        self.Rz = 90
        self.inv_RotationMatrix = np.identity(3)
        self.imu_Rx_Matrix = np.identity(3)
        self.imu_Ry_Matrix = np.identity(3)
        self.imu_Rz_Matrix = np.identity(3)
        self.imu_rotationMatrix = np.identity(3)
        
        rospy.init_node('arm1_update', anonymous=True)
        self.rate = rospy.Rate(15)
        rospy.wait_for_service('AMR1/tm_driver/set_event')
        rospy.wait_for_service('AMR1/tm_driver/set_io')
        rospy.wait_for_service('AMR1/tm_driver/set_positions')

        self.set_event = rospy.ServiceProxy('AMR1/tm_driver/set_event', SetEvent)
        self.set_io = rospy.ServiceProxy('AMR1/tm_driver/set_io', SetIO)
        self.set_positions = rospy.ServiceProxy('AMR1/tm_driver/set_positions', SetPositions)
        self.move_to_initial_joint_angles()

    def move_to_initial_joint_angles(self):
        joint_angles_initial = [0*pi/180, -50*pi/180, 90*pi/180, -50*pi/180, 90*pi/180, 0*pi/180]
        self.set_io(SetIORequest.MODULE_ENDEFFECTOR, 1, 0, 1)
        self.set_positions(SetPositionsRequest.PTP_J, joint_angles_initial, pi, 0.3, 0, False)
        self.set_event(SetEventRequest.TAG, 1, 1)
        self.set_event(SetEventRequest.STOP,0,0)

    def sta_callback(self, msg):
        rospy.loginfo(rospy.get_caller_id() + ': %s', msg.subdata)
        if msg.subcmd == '01':
            data = msg.subdata.split(',')
            if data[1] == 'true':
                rospy.loginfo('point (Tag %s) is reached', data[0])

    def imu_callback(self, data):
        try:
            orientation_q = data.orientation
            self.imu_rotationMatrix = R.from_quat([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]).as_matrix()
            self.update_rotation_matrix()
            #self.rate.sleep()
        except Exception as e:
            rospy.logerr(f"IMU callback encountered an error: {e}")

    def update_rotation_matrix(self):
        imu_Rx, imu_Ry, imu_Rz = R.from_matrix(self.imu_rotationMatrix).as_euler('xyz', degrees=True)
        
        threshold = 1

        print(f'imu_Rx:{imu_Rx}')
        print(f'imu_Ry:{imu_Ry}')
        print(f'imu_Rz:{imu_Rz}')
        print('=============================================')
        if not(abs(imu_Rx) > threshold or abs(imu_Ry) > threshold):
            return 

        cos_Rx = cos(imu_Rx * pi / 180)
        sin_Rx = sin(imu_Rx * pi / 180)
        cos_Ry = cos(imu_Ry * pi / 180)
        sin_Ry = sin(imu_Ry * pi / 180)

        self.imu_Rx_Matrix = np.array([[1, 0, 0],
                                        [0, cos_Rx, -sin_Rx],
                                        [0, sin_Rx, cos_Rx]])

        self.imu_Ry_Matrix = np.array([[cos_Ry, 0, sin_Ry],
                                        [0, 1, 0],
                                        [-sin_Ry, 0, cos_Ry]])

        imu_rotationMatrix = np.dot(self.imu_Rx_Matrix, self.imu_Ry_Matrix)
        self.inv_RotationMatrix = np.linalg.inv(imu_rotationMatrix)

    def EE_callback(self, data):
        global imu_Rx,imu_Ry
        try:
            EEpose_position_vector_new = np.dot(self.inv_RotationMatrix, self.EEpose_position_vector)
            if np.all(np.abs(EEpose_position_vector_new - self.EEpose_position_previous) <= np.array([0.001, 0.001, 0.001])):
                return
            else:
                self.move(EEpose_position_vector_new, (self.Rx - imu_Rx)*pi/180, (self.Ry - imu_Ry)*pi/180, self.Rz*pi/180)
                self.EEpose_position_previous = EEpose_position_vector_new
            self.rate.sleep()
        except Exception as e:
            rospy.logerr(f"EE callback encountered an error: {e}")

    def move(self, EEpose_position_vector_new, Rx, Ry, Rz):
        try:
            pose_and_orientation = EEpose_position_vector_new.tolist() + [Rx, Ry, Rz]
            #print(f'pose_and_orientation:{pose_and_orientation}')
            #print('=============================================')
            self.set_positions(SetPositionsRequest.PTP_T, pose_and_orientation, pi, 0.3, 0, False)
            self.set_event(SetEventRequest.TAG, 2, 0)
            self.set_event(SetEventRequest.STOP,0,0)
            self.set_event(SetEventRequest.RESUME, 0, 0)
        except Exception as e:
            rospy.logerr(f"Move function encountered an error: {e}")

def main():
    arm = ListenerArm()
    rospy.Subscriber("AMR1/tm_driver/sta_response", StaResponse, arm.sta_callback)
    rospy.Subscriber("AMR1/imu/data_raw", Imu, arm.imu_callback)
    rospy.Subscriber("AMR1/tool_pose", PoseStamped, arm.EE_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
