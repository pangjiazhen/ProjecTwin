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

# projector初始化，投影仪初始化方向默认以[0, 1, 0]为起始方向，便于计算，影响平面的生成
# set aspect = 9 / 16, ratio = 2.92, vector [0, 1, 0], width = 0.128, height = 0.072
projector = scene.Projector(aspect=9 / 16, ratio=1.37, vector = [0, 1, 0], width= 0.128, height=0.072)
projector.transform([0,0,0,0,0,0])
# observer 初始化
observer = scene.Observer()
cobot = rc.Moblierobot()

# 构建 模型
mengpi = gm.SamplingMengpi(center = [0,1.12],mengpi_radius=1.5, angle_start = -39 - 90, angle_end = 39 - 90, mengpi_height=1.5)
mengpi.pointsampling(angle_step = 3,z_step = 0.05)
mengpi.samplingcloud=mengpi.samplingcloud+[0,0,0.25]
# 模型 采样

# 进行 投影 采样
grid = mengpi.samgeneration(id=684, x_grids=5, y_grids=5, intervallength=0.05, intervalangle=3)
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
human_radius = 0.8
human_pos = [-0.8,-1,0]
pfield.rt2_block(type = 'cylinder',pos = human_pos, radius = human_radius, center = midpoint)
print (len(pfield.pointcloud))


# 条件3 投影
pfield.rt3_cast(grid, projector)
ans = False
print (len(pfield.pointcloud))
pfield.pointcloud = np.array(pfield.pointcloud)


# 条件4 机器人执行, getposition 是变成六元数，使用projector是因为是默认坐标系的缘故
# pfield.getposition(grid, projector)
print (len(pfield.pointcloud))
if len(pfield.pointcloud)>0:
    area, point, move, joint, jscore, maxcore, ans = pfield.rt4_robot(cobot, type = 'movetrans')
else:
    print('No solution')
#
if ans:
    print('sucess!')
print (area, point, move, joint, jscore, maxcore, ans)
print (midpoint)

robot_pos = [point[0]-move[0],point[1]-move[1],0]

#输出含义：area代表最优解，point代表投影仪位置，move代表倒退位置，joint代表最终关节位置，js，max代表评价
# 机器人变换

robot_arm =[]
robot_arm_x = [0.142745376,0.148156077,0.253144205,0.02629821,0.0607597977,0.0727656037]
robot_arm_y = [0,0.0929043293,0.497950554,0.816923618,0.853131175,0.905753255]
robot_arm_z = [-1.37395883,-1.30192411,-1.30981016,-1.2927711,-1.2502284,-1.21222448]

# 输出到unity robot
outjoint = joint*180/3.14
print ('joint for unity:',-(outjoint[0]+180),outjoint[1]+90,outjoint[2],outjoint[3]+90,-outjoint[4],outjoint[5])
print ('position for unity:',point[0]-move[0],point[1]-move[1],point[2],0,point[4],0)


# 生成图像
marker = []
for i in range(5):
    for j in range(5):
        marker.append([(i*7.25+1.76)/32.51, (j*4.25+1.76)/20.55])
projector.transform(point)
projector.receive(grid)

# 图像变换
img = cv2.imread('EX2.png')
marker = np.array(marker)
c_src = [1,1]-marker
c_dst = projector.received
warped = pf.warp_image_cv(img, c_src, c_dst, dshape=(720, 1280))
pf.show_warped(img, warped, c_src, c_dst)
cv2.imwrite('EX3.png', warped)










# 绘图模块
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 点绘图
ax.scatter(grid[:, 0], grid[:, 1], grid[:, 2], c='r', marker='*' , s = 100)
ax.scatter(projector.position[0],projector.position[1],projector.position[2], c='g', marker='o')
ax.scatter(pfield.pointcloud[:, 0],pfield.pointcloud[:, 1],pfield.pointcloud[:, 2], c='g', marker='o')
ax.scatter(midpoint[0],midpoint[1],midpoint[2], c='g', marker='o')
ax.scatter(point[0],point[1],point[2], c='y', marker='*',s = 100)
x = mengpi.samplingcloud[:, 0]
y = mengpi.samplingcloud[:, 1]
z = mengpi.samplingcloud[:, 2]
ax.scatter(x, y, z, c='b', marker='o')


# 圆绘图

# 根据人画图生成圆上的点，100时分辨率
theta = np.linspace(0, 2 * np.pi, 100)
human_x = human_pos[0] + human_radius/2 * np.cos(theta)
human_y = human_pos[1] + human_radius/2 * np.sin(theta)
human_z = human_pos[2] * np.ones_like(theta)

robot_x = robot_pos[0] + human_radius/2 * np.cos(theta)
robot_y = robot_pos[1] + human_radius/2 * np.sin(theta)
robot_z = robot_pos[2] * np.ones_like(theta)


# 绘制圆
ax.plot(human_x, human_y, human_z)
ax.plot(robot_x, robot_y, robot_z, c = 'b')




# 绘制线
# 机器人绘图
ax.plot(robot_arm_x,  robot_arm_z, robot_arm_y,'-',linewidth = 5)
ax.scatter(robot_arm_x,  robot_arm_z, robot_arm_y,'.',s = 20,c = 'r')
# 热力点绘图，用x,y,z

# scatter = ax.scatter(x, y, z, c=values, cmap='jet')
# fig.colorbar(scatter)  # 添加颜色条



plt.gca().set_box_aspect((2, 1.5, 1.6))
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Point Cloud on Surface')
plt.show()


