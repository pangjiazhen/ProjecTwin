import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 生成示例数据
x = np.random.randn(100)
y = np.random.randn(100)
z = np.random.randn(100)
values = np.random.rand(100)  # 范围在0到1之间的随机值，用于表示颜色

# 创建3D图形对象
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制散点图
scatter = ax.scatter(x, y, z, c=values, cmap='hot')
fig.colorbar(scatter)  # 添加颜色条

# 设置图形标题和轴标签
ax.set_title('3D Scatter Plot with Color Mapping')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# 显示图形
plt.show()