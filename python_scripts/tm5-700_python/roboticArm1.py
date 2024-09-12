#!/usr/bin/env python3

import rospy
from math import pi, cos, sin, radians, degrees
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from tm_msgs.msg import *
from tm_msgs.srv import *
from arm_pose.msg import CoordinatesTheta
from TM700_IK import inverse

imu_Rx = 0
imu_Ry = 0

class ListenerArm:
    def __init__(self):
        self.EEpose_position_vector = np.array([.488745, .1223, .756169])
        self.EEpose_position_previous = np.array([.488745, .1223, .756169])
        self.q_previos = np.array([180, 50, -90, 50, -90, 0])
        self.Rx = 80
        self.Ry = 0
        self.Rz = 90
        self.RotationMatrix = np.identity(3)
        self.imu_Rx_Matrix = np.identity(3)
        self.imu_Ry_Matrix = np.identity(3)
        self.imu_Rz_Matrix = np.identity(3)
        self.imu_rotationMatrix = np.identity(3)
        rospy.init_node('arm1_update', anonymous=True)
        self.rate = rospy.Rate(2)
        self.r_z = 0
        self.delta_x = 0
        self.delta_y = 0
        self.delta_z_1 = 0
        self.delta_z_2 = 0 
        rospy.wait_for_service('AMR1/tm_driver/set_event')
        rospy.wait_for_service('AMR1/tm_driver/set_io')
        rospy.wait_for_service('AMR1/tm_driver/set_positions')

        self.set_event = rospy.ServiceProxy('AMR1/tm_driver/set_event', SetEvent)
        self.set_io = rospy.ServiceProxy('AMR1/tm_driver/set_io', SetIO)
        self.set_positions = rospy.ServiceProxy('AMR1/tm_driver/set_positions', SetPositions)
        self.move_to_initial_joint_angles()

    def move_to_initial_joint_angles(self):
        joint_angles_initial = np.radians(np.array([180, 50, -90, 50, -90, 0])).tolist()
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

    def amr_pose_callback(self, data):
        self.delta_x, self.delta_y ,self.delta_z_1, self.delta_z_2, self.r_z = data.x, data.y, data.z_1, data.z_2, data.r_z
        # print(f'delta_x:{self.delta_x}')
        # print(f'delta_y:{self.delta_y}')
        # print(f'delta_z_1:{self.delta_z_1}')
        # print(f'delta_z_2:{self.delta_z_2}')
        # print(f'Rz_1WithRespectTo2:{self.r_z}')
        # print('=============================================')

    def update_rotation_matrix(self):
        imu_Rx, imu_Ry, imu_Rz = R.from_matrix(self.imu_rotationMatrix).as_euler('xyz', degrees=True)
        
        threshold = 1

        #print(f'imu_Rx:{imu_Rx}')
        #print(f'imu_Ry:{imu_Ry}')
        #print(f'imu_Rz:{imu_Rz}')
        #print('=============================================')
        if not(abs(imu_Rx) > threshold or abs(imu_Ry) > threshold):
            return 

        cos_Rx = cos(radians(imu_Rx))
        sin_Rx = sin(radians(imu_Rx))
        cos_Ry = cos(radians(imu_Ry))
        sin_Ry = sin(radians(imu_Ry))
        cos_Rz = cos(-self.r_z)
        sin_Rz = sin(-self.r_z)

        self.imu_Rx_Matrix = np.array([[1, 0, 0],
                                        [0, cos_Rx, -sin_Rx],
                                        [0, sin_Rx, cos_Rx]])

        self.imu_Ry_Matrix = np.array([[cos_Ry, 0, sin_Ry],
                                        [0, 1, 0],
                                        [-sin_Ry, 0, cos_Ry]])
        
        self.Rz_Matrix = np.array([[cos_Rz, -sin_Rz, 0],
                                   [sin_Rz, cos_Rz, 0],
                                   [0, 0, 1]])

        imu_rotationMatrix = np.dot(self.imu_Rx_Matrix, self.imu_Ry_Matrix)
        inv_RotationMatrix = np.linalg.inv(imu_rotationMatrix)
        self.RotationMatrix = np.dot(inv_RotationMatrix, self.Rz_Matrix)

    def EE_callback(self, data):
        global imu_Rx,imu_Ry
        try:
            #translation fisrt, rotaion second
            move_Command = np.array([0, (abs(self.delta_y) - 1.2)/2, 0]) - np.array([0, 0, (self.delta_z_1 - self.delta_z_2)/2])
            EEpose_position_vector_new = np.dot(self.RotationMatrix, self.EEpose_position_vector + move_Command)
            
            if np.all(np.abs(EEpose_position_vector_new - self.EEpose_position_previous) <= np.array([0.001, 0.001, 0.001])):
                return
            else:
                self.move(EEpose_position_vector_new.tolist(), radians(self.Rx - imu_Rx), radians(self.Ry - imu_Ry), radians(self.Rz))
                self.EEpose_position_previous = EEpose_position_vector_new
            self.rate.sleep()
        except Exception as e:
            rospy.logerr(f"EE callback encountered an error: {e}")

    def move(self, EEpose_position_vector_new, Rx, Ry, Rz):    
        min_norm = float('inf')
        best_solution = None
        try:
            pose_and_orientation = EEpose_position_vector_new + [Rx, Ry, Rz]
            print(f'EE_position:{EEpose_position_vector_new}')
            print(f'EE_Rx:{Rx}')
            print(f'EE_Ry:{Ry}')
            print(f'EE_Rz:{Rz}')
            print('=============================================')
            num_sols, q_sols = inverse(EEpose_position_vector_new, degrees(Rx), degrees(Ry), degrees(Rz))
            if num_sols == 0:
                print("exceed")
                return
         
            elif num_sols !=0:
                for i in range(len(q_sols)):
                    if -45 <= q_sols[i][0] <= 225:
                        joint_change_norm = np.linalg.norm(q_sols[i] - self.q_previos)
                        if joint_change_norm < min_norm:
                            min_norm = joint_change_norm
                            best_solution = q_sols[i]
                self.q_previos = best_solution
                print("check")
                self.set_positions(SetPositionsRequest.PTP_J, np.round(np.radians(best_solution),3).tolist(), pi, 0.3, 0, False)
                print(np.round(np.radians(best_solution),3).tolist())
                self.set_event(SetEventRequest.TAG, 2, 0)
                self.set_event(SetEventRequest.STOP, 0, 0)
        except Exception as e:
            rospy.logerr(f"Move function encountered an error: {e}")

def main():
    arm = ListenerArm()
    rospy.Subscriber("AMR1/tm_driver/sta_response", StaResponse, arm.sta_callback)
    rospy.Subscriber("AMR1/imu/data_raw", Imu, arm.imu_callback)
    rospy.Subscriber("/armPose3D", CoordinatesTheta, arm.amr_pose_callback)
    rospy.Subscriber("AMR1/tool_pose", PoseStamped, arm.EE_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
