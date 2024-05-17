import projectionscene as scene
import fun_projection as fp
import visualization as vl
import fun_fieldsolve as fs
import matplotlib.pyplot as plt
import numpy as np
# 场景初始化

# projector初始化，投影仪初始化方向默认以[0, 1, 0]为起始方向，便于计算，影响平面的生成
# set aspect = 9 / 16, ratio = 2.92, vector [0, 1, 0], width = 0.128, height = 0.072
projector = scene.Projector(aspect=9 / 16, ratio=1.37, vector = [0, 1, 0], width= 0.128, height=0.072)
# object 初始化
#file_path = r'C:\Users\Lenovo\Desktop\experiments\planepart.obj'
#obj = scene.Object(file_path)

# observer 初始化
observer = scene.Observer()

# 开始投影
# 生成域

# 校正场景
# inputplane = [([-0.17123288,  0.,  0.90368151]),
#                 ([0.17123288, 0., 0.90368151]),
#                 ([-0.17123288,  0.,  1.09631849]),
#                 ([0.17123288, 0., 1.09631849])]
inputplane = [([-0.15,  0.,  1.52]),
                ([0.15, 0., 1.52]),
                ([-0.15,  0.,  1.68]),
                ([0.15, 0., 1.68])]


# 初始化
pfield = fs.Pfield()
# 条件1 焦距问题,以半径为r生成1000个视点，r=1.5
pfield.rt1_range(num=1000,p = [0,0,1.6], r= 1)

# 条件2 机器人运动可行域
pfield.rt2_robot(hmax=2.8,hmin=1.2)

# 条件3 遮挡域
pfield.rt3_block(type = 'cylinder',pos = [1,-3,0], radius = 0.1, center = [0,0,0])

# 条件4 能否投影
pfield.rt4_cast(inputplane, projector)

# 条件5 观察者是否能看到，暂时不考虑
# pfield.rt5_visual(position = [0,-1,1,0,0,0])

# 将点云生成为position的形式
pfield.getposition(inputplane, projector)

# 机器人解算
robot_position = [0, -1.2, 1.1, 0, 0, 0]
cobot = scene.Moblierobot()
# print (fp.rad2angle(endjoint), score, answer)

test_joint1 = []
test_score1 = []
test_ans1 = []
test_position1 = []
test_max1 = []
test_joint2 = []
test_score2 = []
test_ans2 = []
test_max2 = []
test_position2 = []
test_joint3 = []
test_score3 = []
test_ans3 = []
test_max3 = []
test_position3 = []
sum1 = 0
sum2 = 0
sum3 = 0
for position in pfield.position:
    relevant_position = np.array(position) - robot_position
    [endjoint, score, maxscore, answer] = cobot.s1_fixed(relevant_position)
    if answer:
        test_joint1.append(fp.robot_angle(endjoint))
        test_score1.append(score)
        test_max1.append(maxscore)
        test_position1.append(position)
        sum1 =sum1+1

for position in pfield.position:
    relevant_position = np.array(position) - robot_position
    [endjoint, score, maxscore, answer] = cobot.s2_move(relevant_position)
    if answer:
        test_joint2.append(fp.robot_angle(endjoint))
        test_score2.append(score)
        test_max2.append(maxscore)
        test_position2.append(position)
        sum2 = sum2 + 1

for position in pfield.position:
    relevant_position = np.array(position) - robot_position
    [endjoint, score, maxscore, answer] = cobot.s3_movetrans(relevant_position)
    if answer:
        test_joint3.append(fp.robot_angle(endjoint))
        test_score3.append(score)
        test_max3.append(maxscore)
        test_position3.append(position)
        sum3 = sum3 + 1


print(np.mean(test_score1),np.mean(test_score2),np.mean(test_score3))
print(np.mean(test_max1),np.mean(test_max2),np.mean(test_max3))
print (len(test_score1),len(test_score2),len(test_score3))
# relevant_position = [0, 0.4,0.5,0,0,0]
# [endjoint, score, answer] = cobot.s1_fixed(relevant_position)
# test_joint.append(fp.robot_angle(endjoint))
# test_score.append(score)
# test_position.append([position(0),position(1),position(2)])
# test_position = np.array(test_position)
#
# projector.transform([-0.7,-0.7,1.6,0,45,0])
#

points = np.array(pfield.pointcloud)
test_position1 = np.array(test_position1)
test_position2 = np.array(test_position2)
fig2 = plt.figure("sphere points uniform")
ax2 = fig2.add_subplot(111, projection='3d')
ax2.scatter(points[:, 0], points[:, 1], points[:, 2], color='r')
ax2.scatter(test_position1[:, 0], test_position1[:, 1], test_position1[:, 2], color='b')
ax2.scatter(test_position2[:, 0], test_position2[:, 1], test_position2[:, 2], color='g')


plt.show()
# #observer.transform(0,1,-1,0,0,0)
# # 投影校正
#
#
#
# projector.receive(inputplane)
# print(projector.received, np.min(projector.received),np.max(projector.received))
#
# # 图像生成
# vl.imcorrection(projector.received,'figure1-2.png','img_correction.png')
#

