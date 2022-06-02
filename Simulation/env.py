import sim
import numpy as np
import math
import time
from math import cos, sin, radians, degrees

np.set_printoptions(suppress=True)


class Dobot:

    resetJointAngles = np.array([[0, 0, 0, 0]])
    baseName = 'Dobot'
    jointName = 'motor'
    jointNums = 4

    def __init__(self):
        jointNums = self.jointNums
        baseName = self.baseName
        jointName = self.jointName

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
        joint_handle = np.zeros(jointNums, dtype=np.int32)  # 注意需要定义为向量
        joint_config = np.zeros(jointNums, dtype=np.float32)

        for i in range(jointNums):
            _, return_handle = sim.simxGetObjectHandle(clientID, './'+jointName+str(i+1), sim.simx_opmode_blocking
                                                       )
            joint_handle[i] = return_handle
        _, base_handle = sim.simxGetObjectHandle(
            clientID, baseName, sim.simx_opmode_blocking)

        for i in range(jointNums):
            _, jpos = sim.simxGetJointPosition(
                clientID, joint_handle[i], sim.simx_opmode_blocking)
            joint_config[i] = jpos

        self.clientID = clientID
        self.joint_handle = joint_handle
        self.joint_config = joint_config
        self.operator = 'suctionCup'
        self.ControlRobotJoints(self.resetJointAngles)
        time.sleep(2)

    def showHandles(self):

        jointNum = self.jointNums
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

        jointNum = self.jointNums
        joint_handle = self.joint_handle
        clientID = self.clientID

        for i in range(jointNum):
            _, jpos = sim.simxGetJointPosition(
                clientID, joint_handle[i], sim.simx_opmode_blocking)
            print(round(self.radianTOangle(float(jpos)), 2))
        print('\n')

    def openSuctionCup(self):
        clientID = self.clientID
        sim.simxSetIntegerSignal(
            clientID, self.operator, True, sim.simx_opmode_oneshot)

    def closeSuctionCup(self):
        clientID = self.clientID
        sim.simxSetIntegerSignal(
            clientID, self.operator, False, sim.simx_opmode_oneshot)

    def TestRobotRun(self, num, angle):
        clientID = self.clientID
        joint_handle = self.joint_handle
        sim.simxSetJointTargetPosition(
            clientID, joint_handle[num], radians(angle), sim.simx_opmode_oneshot)
        self.DelayCommand()

    def ControlRobotJoints(self, jointAngles):
        clientID = self.clientID
        joint_handle = self.joint_handle
        for i in range(self.jointNums):
            sim.simxSetJointTargetPosition(clientID, joint_handle[i], radians(
                jointAngles[0, i]), sim.simx_opmode_oneshot)
        self.DelayCommand()

    def DelayCommand(self):  # 执行完一个指令后必须加延时函数
        time.sleep(0.00000000000001)


class Kinematics:
    def __init__(self):  # 仅考虑旋转关节
        self.DH_alpha = np.array([0, 90, 0, 90]).reshape((1, 4))
        self.DH_a = np.array([0, 0, 0.135, 0.2067]).reshape((1, 4))
        self.DH_d = np.array([0, 0, 0, 0]).reshape((1, 4))

    # DH建模后相邻两杆的单级齐次变换矩阵（采用MDH）
    def T_mat_DH(self, alpha, a, d, theta):
        alpha = radians(alpha)
        theta = radians(theta)  # 由角度变为弧度
        matrix = np.mat(np.zeros((4, 4)))
        matrix[0, 0] = cos(theta)
        matrix[0, 1] = -sin(theta)
        matrix[0, 3] = a
        matrix[1, 0] = sin(theta) * cos(alpha)
        matrix[1, 1] = cos(theta) * cos(alpha)
        matrix[1, 2] = -sin(alpha)
        matrix[1, 3] = -sin(alpha) * d
        matrix[2, 0] = sin(theta) * sin(alpha)
        matrix[2, 1] = cos(theta) * sin(alpha)
        matrix[2, 2] = cos(alpha)
        matrix[2, 3] = cos(alpha) * d
        matrix[3, 3] = 1
        return matrix

    def DOF4_matrix(self, jointsAngles):
        DH_theta = np.array(jointsAngles).reshape((1, 4))
        DH_Parameter_Table = np.concatenate(
            (self.DH_alpha, self.DH_a, self.DH_d, DH_theta), axis=0)
        DH_mat = DH_Parameter_Table.T
        DOF4_mat = np.identity(4)
        for i in range(0, 4):
            temp_mat = self.T_mat_DH(
                DH_mat[i, 0], DH_mat[i, 1], DH_mat[i, 2], DH_mat[i, 3])
            DOF4_mat = DOF4_mat * temp_mat
        return DOF4_mat

    def ForwardKinematics(self, jointsAngles):
        dof4Mat = self.DOF4_matrix(jointsAngles)
        x = dof4Mat[0, 3]
        y = dof4Mat[1, 3]
        z = dof4Mat[2, 3]
        return [x, y, z]

    def InverseKinematics(self, eePos):
        # NOTE: 注意多解问题
        px, py, pz = eePos[0], eePos[1], eePos[2]
        a2 = 0.135
        a3 = 0.2067
        jointAngles = np.zeros((1, 4))
        theta1 = math.atan2(py, px)

        if degrees(theta1) <= 90 and degrees(theta1) >= -90:
            print('theta1 is ok!')
        else:
            print('theta1 is bad!')
            return
        c1 = math.cos(theta1)
        s1 = math.sin(theta1)
        c3 = (pz**2+(px*c1+py*s1)**2-a2**2-a3**2)/(2*a2*a3)
        if (1-c3**2) < 0:
            print("error!,s3 little problem!")
            s3 = 0
        else:
            s3 = math.sqrt(1-c3**2)

        theta3 = math.atan2(-s3, c3)  # 确保theta3在范围内 根据关节运动范围s3取负，这个是根据DH建系来看的
        if degrees(theta3) <= 90 and degrees(theta3) >= -10:
            print('theta3 is ok!')
        else:
            print('theta3 is bad!')
            return
        # 从实际建系以及机器人运动来看，theta2为正角
        c2 = (-pz*a3*s3+(px*c1+py*s1)*(a2+a3*c3)) / \
            ((-a3*s3)**2+(a2+a3*c3)**2) 
        if 1-c2**2 < 0:
            print("error!,s2 little problem!")
            s2 = 0
        else:
            s2 = math.sqrt(1-c2**2)
        theta2 = math.atan2(s2, c2)
        if degrees(theta2) <= 85 and degrees(theta1) >= 0:
            print('theta2 is ok!')
        else:
            print('theta2 is bad!')
            return

        theta4 = 0
        jointAngles[0, 0] = round(degrees(theta1), 3)
        jointAngles[0, 1] = round(degrees(theta2), 3)
        jointAngles[0, 2] = round(degrees(theta3), 3)
        jointAngles[0, 3] = round(degrees(theta4), 3)

        return jointAngles


if __name__ == '__main__':
    robot = Dobot()
    kinematics = Kinematics()
    eePos = kinematics.ForwardKinematics([20, 0, 0, 0])
    print(eePos)
    jointAngles = kinematics.InverseKinematics(eePos)
    if (math.fabs(eePos[0]) < 1e-5) and (math.fabs(eePos[1] < 1e-5)):  # 这种情况本来就没有意义
        jointAngles = np.array([[0, 90, 0, 0]])
    print(jointAngles)
    robot.ControlRobotJoints(jointAngles)
