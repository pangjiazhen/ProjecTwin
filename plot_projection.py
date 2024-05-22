from roboticstoolbox import SE3

# 创建一个SE3刚体变换
T = SE3.Rx(0.5) * SE3(1, 2, 3)

# 获取旋转矩阵和平移向量
R = T.R
t = T.t

# 打印刚体变换的旋转矩阵和平移向量
print("Rotation matrix:")
print(R)
print("Translation vector:")
print(t)

# 利用刚体变换进行点坐标变换
p = [1, 0, 0]
p_transformed = T * p

# 打印变换后的点坐标
print("Transformed point:")
print(p_transformed)