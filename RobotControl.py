import math
import roboticstoolbox as rtb
import numpy as np
import ProjectionFun as fp

class Moblierobot:
    def __init__(self):
        self.robot = rtb.models.DH.UR5()
        pose = np.array([0, 0, 0, 0, 0, 0]) * math.pi / 180
        joint = [0, 0, 0, 0, 0, 0]
        # 暂时令机器人处于zyx坐标系中
        joint[0] = -pose[0] - 180 * math.pi / 180
        joint[1] = pose[1] - 90 * math.pi / 180
        joint[2] = pose[2]
        joint[3] = pose[3] - 90 * math.pi / 180
        joint[4] = -pose[4]
        joint[5] = pose[5]
        self.startjoint = joint
        self.matrix = np.array(self.robot.fkine(joint))
        self.start = pose

    def iksolution(self,position):
        #先进行旋转
        aa = self.matrix
        subm1 = np.dot(self.matrix[:3, :3], fp.rotation_angle(-position[3],position[4],position[5]))
        #在进行移动
        subm2 = np.reshape(np.array(position[0:3]), (-1, 1))
        subm3 = self.matrix[3,:]
        #最后拼接
        matrix_with_extra_column = np.hstack((subm1, subm2))
        matrix_with_extra_row = np.vstack((matrix_with_extra_column, subm3))
        endjoint = self.robot.ikine_LM(matrix_with_extra_row, q0 = self.startjoint).q
        answer = self.robot.ikine_LM(matrix_with_extra_row, q0 = self.startjoint).success
        return endjoint, answer

    def jointeva(self, endjoint):
        jscore = np.linalg.norm(self.startjoint - endjoint, ord=1)
        maxscore = np.max(endjoint)
        return jscore, maxscore
    def s1_fixed(self, position):
        move_position = [0,0,0]
        [endjoint, answer] = self.iksolution(position)
        [jscore, maxscore] = self.jointeva(endjoint)
        return move_position, endjoint, jscore, maxscore, answer

    def s2_move(self, position):
        move_position = position
        angle_radians = math.radians(position[4])
        sine_value = math.sin(angle_radians)
        cose_value = math.cos(angle_radians)
        move_position[0] = -sine_value * 0.2
        move_position[1] = -cose_value * 0.2
        [endjoint, answer] = self.iksolution(move_position)
        [jscore, maxscore] = self.jointeva(endjoint)
        return move_position[0:3], endjoint, jscore, maxscore, answer

    def s3_movetrans(self, position):
        move_position = position.copy()
        agv_position = position.copy()
        angle_radians = math.radians(position[4])
        sine_value = math.sin(angle_radians)
        cose_value = math.cos(angle_radians)
        agv_position[0] = sine_value * 0.2
        agv_position[1] = cose_value * 0.2
        move_position[0] = 0
        move_position[1] = 0.2
        move_position[4] = 0
        [endjoint, answer] = self.iksolution(move_position)
        [jscore, maxscore] = self.jointeva(endjoint)
        return agv_position[0:3], endjoint, jscore, maxscore, answer