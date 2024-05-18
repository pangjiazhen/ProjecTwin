import ProjectionScene as scene
import Model_mengpi as gm
import ProjectionFun as fp
import visualization as vl
import FieldFun as fs
import matplotlib.pyplot as plt
import numpy as np
# 场景初始化

# projector初始化，投影仪初始化方向默认以[0, 1, 0]为起始方向，便于计算，影响平面的生成
# set aspect = 9 / 16, ratio = 2.92, vector [0, 1, 0], width = 0.128, height = 0.072
projector = scene.Projector(aspect=9 / 16, ratio=1.37, vector = [0, 1, 0], width= 0.128, height=0.072)
projector.transform([0,-3,0.75,0,0,0])
# observer 初始化
observer = scene.Observer()


# 构建 模型
mengpi = gm.SamplingMengpi(center = [0,1.12],mengpi_radius=1.5, angle_start = -41.5 - 90, angle_end = 41.5 - 90, mengpi_height=1.5)
mengpi.pointsampling(angle_step = 5,z_step = 0.05)
# 模型 采样
# 模型 取区域

print(mengpi.samplingcloud)
print(mengpi.samplingcloud.shape[0])
check = []
for i in range(mengpi.samplingcloud.shape[0]):
    grid = mengpi.samgeneration(id = i, x_grids=5, y_grids=5, intervallength=0.05, intervalangle=3)
    grid = np.array(grid)
    projector.receive(grid)
    ans = projector.evaluate()
    check.append(ans)

print (projector.received)

# 绘图模块
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 点绘图
ax.scatter(grid[:, 0], grid[:, 1], grid[:, 2], c='r', marker='o')
ax.scatter(projector.position[0],projector.position[1],projector.position[2], c='r', marker='o')

# 热力点绘图
x = mengpi.samplingcloud[:, 0]
y = mengpi.samplingcloud[:, 1]
z = mengpi.samplingcloud[:, 2]
values = check
scatter = ax.scatter(x, y, z, c=values, cmap='jet')
fig.colorbar(scatter)  # 添加颜色条



ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Point Cloud on Surface')
plt.show()


