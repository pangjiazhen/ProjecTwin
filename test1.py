import roboticstoolbox as rtb
from spatialmath import SE3
import math
import numpy as np
import fun_projection as fp
# Make a Panda robot
robot = rtb.models.DH.UR5()
pose = np.array([0,0,0,0,0,0]) *math.pi/180
joint = [0,0,0,0,0,0]

# 暂时令机器人处于zyx坐标系中
joint[0] = -pose[0]- 180 * math.pi/180
joint[1] = pose[1] - 90 * math.pi/180
joint[2] = pose[2]
joint[3] = pose[3] - 90 * math.pi/180
joint[4] = -pose[4]
joint[5] = pose[5]

Tep = np.array(robot.fkine(joint))
print (Tep)
#现在状态是[0,1,0]
submatrix = Tep[:3, :3]
temp = np.array(Tep[:3,3])
# temp = np.array([0.04237899,0.190645,1.000001])
submarrix2 = np.reshape(temp, (-1, 1))
# print (submarrix2)
submarrix3 = Tep[3,:]
matrix = fp.rotation_angle(30,45,0)
print (matrix)
ro_ma = np.dot(submatrix,matrix)

#
matrix_with_extra_column = np.hstack((ro_ma, submarrix2))



matrix_with_extra_row = np.vstack((matrix_with_extra_column, submarrix3))

print (matrix_with_extra_row)
TT = robot.ikine_LM(matrix_with_extra_row,q0 = joint)
print (TT)

TTT=fp.robot_angle(TT.q)
print (TTT)
# joint1 = joint1
# angle = np.array([-242.5,-10,-80,90,-120,0])
# angle = angle * math.pi/180
# Teptest = robot.fkine(angle)
# print (robot)
# target = np.array([[1,0,0,0],
#           [0,0,1,0.5],
#           [0,1,0,0],
#             [0,0,0,1]])
# TT = robot.ikine_LM(target,q0=robot.qz)
# print (TT)
# Tep = robot.fkine([0, 0, 0, 0, 0, 0])
# T = robot.fkine(robot.qr)
# print (T)
# qt = rtb.tools.trajectory.jtraj(robot.qz, joint, 50)
# robot.plot(qt.q)
# # T = robot.fkine(robot.qr)
# print (Tep)
# TT = robot.ikine_LM(Tep,q0=robot.qz)
# print (TT.q*180/math.pi)
#
# Tep2 = robot.fkine(TT.q)
# print (Tep2)
# solutions = robot.ikine_LM(Tep, q0=robot.qz, ilimit=100)

# for i, sol in enumerate(solutions):
#     print(f"Solution {i+1}: {sol.q}")
