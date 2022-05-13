import sim
import numpy as np
import math


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

if __name__ == '__main__':
    robot = Dobot()
    robot.TestRobotRun(0,90)
