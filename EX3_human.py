import ProjectionScene as scene
import Model_mengpi as gm
import visualization as vl
import ProjectionFun as pf
import RobotControl as rc
import FieldFun as fs
import matplotlib.pyplot as plt
import numpy as np
import cv2
import math
import roboticstoolbox as rtb
from matplotlib.patches import Circle
# 场景初始化
projector = scene.Projector(aspect=9 / 16, ratio=1.37, vector = [0, 1, 0], width= 0.128, height=0.072)
projector.transform([0,0,0,0,0,0])
# observer 初始化
observer = scene.Observer()
cobot = rc.Moblierobot()
# 进行 投影 采样
grid = ([-0.1, 0, 1.60],
[-0.1, 0, 1.70],
[0.1, 0, 1.60],
[0.1, 0, 1.70])
grid = np.array(grid)
print(len(grid))
midpoint = np.mean(grid, axis=0)

# projector.receive(grid)
# ans = projector.evaluate()

# 生成域 计算个投影

# 初始化
pfield = fs.Pfield()
# 条件1 焦距问题,以半径为r生成1000个视点，r=1.5
pfield.rt1_range(num=1000,p = midpoint, r= 1)
print (len(pfield.pointcloud))



# 条件2 遮挡阈值,人的位置时pos，
human_radius = 0.6
human_pos = [0,-0.5,0]
pfield.rt2_block(type = 'cylinder',pos = human_pos, radius = human_radius, center = midpoint)
print (len(pfield.pointcloud))









robot_arm_x2 = [0.142745376,0.148156077,0.253144205,0.02629821,0.0607597977,0.0727656037]
robot_arm_y2 = [0,0.0929043293,0.497950554,0.816923618,0.853131175,0.905753255]
robot_arm_z2 = [-1.37395883,-1.30192411,-1.30981016,-1.2927711,-1.2502284,-1.21222448]
r1 = np.array([(-1.46100008,1,-1.29493713),
               (-1.42218685,1.09290433,-1.35586166),
(-1.36089528,1.50505209,-1.31681454),
(-1.03046942,1.50505209,-1.10631037),
(-1.00628853,1.55283356,-1.14426672),
(-0.984351277,1.59866369,-1.10212564)])

r2 = np.array([(-0.372281462,1,0.619117022),
              (-0.300318748,1.09290433,0.612821102),
               (-0.280989856,1.44781744,0.833751559),
               (-0.309947252,1.65543032,0.502765596),
               (-0.267320812,1.61490941,0.473619163),
               (-0.296142668,1.59784615,0.416736275)])



robot_arm_x = r1[:,0]+0.4
robot_arm_y = r1[:,1]
robot_arm_z = r1[:,2]+0.3

robot_arm_x2 = r2[:,0]+0.28
robot_arm_y2 = r2[:,1]
robot_arm_z2 = r2[:,2]-0.35




# 绘图模块
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 点绘图
ax.scatter(grid[:, 0], grid[:, 1], grid[:, 2], c='r', marker='*' , s = 100)

ax.scatter(pfield.pointcloud[:, 0],pfield.pointcloud[:, 1],pfield.pointcloud[:, 2], c='g', marker='o')
ax.scatter(-0.55, -0.8,1.75, c='y', marker='*',s = 100)





# 圆绘图

# 根据人画图生成圆上的点，100时分辨率
theta = np.linspace(0, 2 * np.pi, 100)
human_x = human_pos[0] + human_radius/2 * np.cos(theta)
human_y = human_pos[1] + human_radius/2 * np.sin(theta)
human_z = human_pos[2] * np.ones_like(theta)
#
# robot_x = robot_arm_x[0] + human_radius/2 * np.cos(theta)
# robot_y = robot_arm_z[0] + human_radius/2 * np.sin(theta)
# robot_z = robot_arm_z[0] * np.ones_like(theta)
# robot_x2 = robot_arm_x2[0] + human_radius/2 * np.cos(theta)
# robot_y2 = robot_arm_z2[0] + human_radius/2 * np.sin(theta)
# robot_z2 = robot_arm_z2[0] * np.ones_like(theta)
#
# # 绘制圆
ax.plot(human_x, human_y, human_z)
# ax.plot(robot_x, robot_y, 0, c = 'b')
# ax.plot(robot_x2, robot_y2, 0, c = 'b')
#
#
#
# # 绘制线
# # 机器人绘图
# ax.plot(robot_arm_x,  robot_arm_z, robot_arm_y,'-',linewidth = 5)
# ax.scatter(robot_arm_x,  robot_arm_z, robot_arm_y,'.',s = 20,c = 'r')
#
#
# ax.plot(robot_arm_x2,  robot_arm_z2, robot_arm_y2,'-',linewidth = 5)
# ax.scatter(robot_arm_x2,  robot_arm_z2, robot_arm_y2,'.',s = 20,c = 'r')
# 热力点绘图，用x,y,z

# scatter = ax.scatter(x, y, z, c=values, cmap='jet')
# fig.colorbar(scatter)  # 添加颜色条



plt.gca().set_box_aspect((2.5, 1, 2))
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Point Cloud on Surface')
plt.show()


