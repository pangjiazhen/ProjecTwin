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
# 场景初始化

# projector初始化，投影仪初始化方向默认以[0, 1, 0]为起始方向，便于计算，影响平面的生成
# set aspect = 9 / 16, ratio = 2.92, vector [0, 1, 0], width = 0.128, height = 0.072
projector = scene.Projector(aspect=9 / 16, ratio=1.37, vector = [0, 1, 0], width= 0.128, height=0.072)
projector.transform([0,0,0,0,0,0])
# observer 初始化
observer = scene.Observer()
cobot = rc.Moblierobot()

# 构建 模型
mengpi = gm.SamplingMengpi(center = [0,1.12],mengpi_radius=1.5, angle_start = -41.5 - 90, angle_end = 41.5 - 90, mengpi_height=1.5)
mengpi.pointsampling(angle_step = 5,z_step = 0.05)
# 模型 采样

# 进行 投影 采样
grid = mengpi.samgeneration(id=300, x_grids=5, y_grids=5, intervallength=0.05, intervalangle=3)
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
# 条件2 遮挡阈值
pfield.rt2_block(type = 'cylinder',pos = [1,-3,0], radius = 0.1, center = [0,0,0])
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
#输出含义：area代表最优解，point代表投影仪位置，move代表倒退位置，joint代表最终关节位置，js，max代表评价
# 机器人变换

#, 机器人变换动画
qt = rtb.tools.trajectory.jtraj(cobot.startjoint, joint, 50)
# cobot.robot.plot(qt.q)

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
cv2.imwrite('EX2.jpg', warped)


# 绘图模块
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 点绘图
ax.scatter(grid[:, 0], grid[:, 1], grid[:, 2], c='r', marker='o')
ax.scatter(projector.position[0],projector.position[1],projector.position[2], c='g', marker='o')
ax.scatter(pfield.pointcloud[:, 0],pfield.pointcloud[:, 1],pfield.pointcloud[:, 2], c='g', marker='o')
ax.scatter(midpoint[0],midpoint[1],midpoint[2], c='g', marker='o')

# 热力点绘图
x = mengpi.samplingcloud[:, 0]
y = mengpi.samplingcloud[:, 1]
z = mengpi.samplingcloud[:, 2]
ax.scatter(x, y, z, c='r', marker='o')
# scatter = ax.scatter(x, y, z, c=values, cmap='jet')
# fig.colorbar(scatter)  # 添加颜色条



ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Point Cloud on Surface')
plt.show()


