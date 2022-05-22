import sim
import numpy as np
import math
from math import cos,sin,radians

class Dobot:

    joint_angle = [0, 0, 0, 0]
    base_name = 'Dobot'
    joint_name = 'motor'
    joint_nums = 4

    def __init__(self):
        joint_nums = self.joint_nums
        base_name = self.base_name
        joint_name = self.joint_name

        print('Simulation started')

        try:
            sim.simxFinish(-1)  # 关掉之前连接
            clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
            if clientID != -1:
                print("Connected to remote API server!")
            else:
                print("Failed connecting to remote API server")
        except:
            print('Check if CoppeliaSim is open')

        sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)
        print('Simulation start')

        # 读取Base和Joint的句柄
        joint_handle = np.zeros(joint_nums, dtype=np.int32)  # 注意需要定义为向量
        joint_config = np.zeros(joint_nums, dtype=np.float32)
       
        for i in range(joint_nums):
            # print(joint_name+str(i+1))
            _, return_handle = sim.simxGetObjectHandle(clientID, './'+joint_name+str(i+1), sim.simx_opmode_blocking
                                                       )
            joint_handle[i] = return_handle
            # print(return_handle)
        _, base_handle = sim.simxGetObjectHandle(
            clientID, base_name, sim.simx_opmode_blocking)

        for i in range(joint_nums):
            _, jpos = sim.simxGetJointPosition(
                clientID, joint_handle[i], sim.simx_opmode_blocking)
            joint_config[i] = jpos


        self.clientID = clientID
        self.joint_handle = joint_handle
        self.joint_config = joint_config
        self.operator = 'suctionCup'

    def showHandles(self):

        jointNum = self.joint_nums
        joint_handle = self.joint_handle

        print('Handles available!')
        print("==============================================")
        print("Handles:  ")
        for i in range(jointNum):
            print("jointHandle" + str(i+1) + ": " + joint_handle[i])
        print("===============================================")

    def __del__(self):
        clientID = self.clientID
        sim.simxFinish(clientID)
        print('Simulation end')

    def showJointAngles(self):

        jointNum = self.joint_nums
        joint_handle = self.joint_handle
        clientID = self.clientID

        for i in range(jointNum):
            _, jpos = sim.simxGetJointPosition(
                clientID, joint_handle[i], sim.simx_opmode_blocking)
            print(round(self.radianTOangle(float(jpos)), 2))
        print('\n')

    def openRG2(self):

        clientID = self.clientID
        sim.simxSetIntegerSignal(clientID, self.operator, True, sim.simx_opmode_oneshot)

    # close rg2

    def closeRG2(self):
        clientID = self.clientID
        sim.simxSetIntegerSignal(clientID, self.operator, False, sim.simx_opmode_oneshot)

    def TestRobotRun(self,num, angle):
        clientID = self.clientID
        joint_handle = self.joint_handle
        sim.simxSetJointTargetPosition(clientID, joint_handle[num], self.angle2radian(angle), sim.simx_opmode_oneshot)
       
              


    def radian2angle(self, radian):
        angle = radian*(180/math.pi)
        return angle
    
    def angle2radian(self,angle):
        radian=angle*(math.pi/180)
        return radian

class Kinematics:
    def __init__(self): # 仅考虑旋转关节
        self.DH_alpha=np.array([0,90,0,90]).reshape((1,4))
        self.DH_a=np.array([0,0,0.135,0.2067]).reshape((1,4))
        self.DH_d=np.array([0,0,0,0]).reshape((1,4))
        
        
    # DH建模后相邻两杆的单级齐次变换矩阵（采用MDH）
    def T_mat_DH(self,alpha,a,d,theta):
        alpha = radians(alpha)
        theta = radians(theta)              #由角度变为弧度
        matrix = np.mat(np.zeros((4, 4)))   
        matrix[0,0] = cos(theta)
        matrix[0,1] = -sin(theta)
        matrix[0,3] = a
        matrix[1,0] = sin(theta) * cos(alpha)
        matrix[1,1] = cos(theta) * cos(alpha)
        matrix[1,2] = -sin(alpha)
        matrix[1,3] = -sin(alpha) * d
        matrix[2,0] = sin(theta) * sin(alpha)
        matrix[2,1] = cos(theta) * sin(alpha)
        matrix[2,2] = cos(alpha)
        matrix[2,3] = cos(alpha) * d
        matrix[3,3] = 1
        return matrix

    def DOF4_matrix(self,joints_angle):
        DH_theta=np.array(joints_angle).reshape((1,4))
        DH_Parameter_Table=np.concatenate((self.DH_alpha,self.DH_a,self.DH_d,DH_theta),axis=0)
        DH_mat = DH_Parameter_Table.T
        print(DH_mat)
        DOF4_mat = np.identity(4)
        for i in range(0,4):
            temp_mat = self.T_mat_DH(DH_mat[i,0],DH_mat[i,1],DH_mat[i,2],DH_mat[i,3])
            DOF4_mat = DOF4_mat * temp_mat
        return DOF4_mat

    def ForwardKinematics(self,joints_angle):
        DOF4_mat=self.DOF4_matrix(joints_angle)
        x=DOF4_mat[0,3]
        y=DOF4_mat[1,3]
        z=DOF4_mat[2,3]
        return [x,y,z]

    def InverseKinematics(self,ee_pos):
        joints_angles=1
        return joints_angles

if __name__ == '__main__':
    # robot = Dobot()
    # robot.TestRobotRun(0,90)
    kinematics=Kinematics()
    position=kinematics.ForwardKinematics([0,0,0,0])
    print(position)