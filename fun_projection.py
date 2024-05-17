import math
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import thinplate as tps
import cv2

from scipy.spatial import ConvexHull

def calculate_max_convex_hull_area(points):
    # 将点集的坐标转换为 NumPy 数组
    points_array = np.array(points)

    # 计算点集的凸包
    hull = ConvexHull(points_array)

    # 获取凸包的顶点坐标
    hull_points = points_array[hull.vertices]

    # 使用 Shoelace 公式计算凸包的面积
    n = len(hull_points)
    area = 0
    for i in range(n):
        x1, y1 = hull_points[i]
        x2, y2 = hull_points[(i + 1) % n]
        area += x1 * y2 - x2 * y1

    return abs(area) / 2

def robot_angle (joint):
    joint[0] = -joint[0] * 180 / math.pi + 180
    joint[1] = joint[1] * 180 / math.pi + 90
    joint[2] = joint[2] * 180 / math.pi
    joint[3] = joint[3] * 180 / math.pi + 90
    joint[4] = -joint[4] * 180 / math.pi
    joint[5] = joint[5] * 180 / math.pi
    return joint

def read_obj_file(file_path):
    # 用于读入真实物体表面数据，采用三角网格的形式
    vertices = []
    normals = []
    triangles = []

    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('v '):
                # 读取顶点数据
                vertex = list(map(float, line.strip().split()[1:]))
                vertices.append(vertex)
            elif line.startswith('vn '):
                # 读取法线数据
                normal = list(map(float, line.strip().split()[1:]))
                normals.append(normal)
            elif line.startswith('f '):
                # 读取三角面片数据
                face = line.strip().split()[1:]
                triangle = []
                for vertex in face:
                    vertex_indices = vertex.split('//')[0]
                    triangle.append(int(vertex_indices))
                triangles.append(triangle)

    return vertices, normals, triangles


# 生成测试数据，球的表面
def sample_points_on_sphere(radius, num_samples):
    points = []
    phi_values = np.linspace(0, 2 * np.pi, num_samples[0])
    theta_values = np.linspace(0, np.pi, num_samples[1])

    for phi in phi_values:
        for theta in theta_values:
            x = radius * np.sin(theta) * np.cos(phi)
            y = radius * np.sin(theta) * np.sin(phi)
            z = radius * np.cos(theta)
            points.append([x, y, z])

    return points


# In[4]:


def closestpoints(line_origin, line_direction, point_cloud):
    # 向量的单位方向向量

    line_direction = line_direction / np.linalg.norm(line_direction)

    # 计算点到直线的距离
    differences = point_cloud - line_origin
    perpendicular_distances = np.linalg.norm(np.cross(differences, line_direction), axis=1)

    # 找到最短距离对应的索引
    nearest_index = np.argmin(perpendicular_distances)
    nearest_point = point_cloud[nearest_index]

    # 记录：如果有需要再加入距离约束
    # nearest_point = find_nearest_points(line_origin, nearest_points)

    return nearest_point


# In[5]:


def sample_rectangular_points(point, width, height, num_samples):
    # 计算矩形的边向量
    point = np.array(point)
    edge1 = width
    edge2 = height

    vector1 = np.array([1, 0, 0])
    vector2 = np.array([0, 0, 1])
    # 计算边向量的步长
    step1 = edge1 / (num_samples[0] - 1)
    step2 = edge2 / (num_samples[1] - 1)

    # 生成网格点坐标
    grid_points = np.zeros((num_samples[0] * num_samples[1], 3))
    index = 0

    for i in range(num_samples[0]):
        for j in range(num_samples[1]):
            grid_points[index] = point + vector2 * step1 * i + vector1 * step2 * j
            index += 1

    return grid_points


# In[6]:


def transform_cam(point, normal, rotation_matrix, pointcloud, length):
    Rotation_point = np.dot(rotation_matrix, pointcloud.T).T
    translation = normal * length
    Rimage = Rotation_point + translation + point

    return Rimage


# In[7]:


def transform_proj(point, normal, rotation_matrix, pointcloud, length):
    translation = normal * length
    pointcloud = pointcloud - translation - point
    Rproj = np.dot(rotation_matrix, pointcloud.T).T

    return Rproj


# In[8]:


def rotation_angle(roll, pitch, yaw):
    # 示例欧拉角（以度数为单位）
    # 将度数转换为弧度
    roll_rad = np.radians(roll)
    pitch_rad = np.radians(pitch)
    yaw_rad = np.radians(yaw)

    # 计算四元数
    qx = np.sin(roll_rad / 2) * np.cos(pitch_rad / 2) * np.cos(yaw_rad / 2) - np.cos(roll_rad / 2) * np.sin(
        pitch_rad / 2) * np.sin(yaw_rad / 2)
    qy = np.cos(roll_rad / 2) * np.sin(pitch_rad / 2) * np.cos(yaw_rad / 2) + np.sin(roll_rad / 2) * np.cos(
        pitch_rad / 2) * np.sin(yaw_rad / 2)
    qz = np.cos(roll_rad / 2) * np.cos(pitch_rad / 2) * np.sin(yaw_rad / 2) - np.sin(roll_rad / 2) * np.sin(
        pitch_rad / 2) * np.cos(yaw_rad / 2)
    qw = np.cos(roll_rad / 2) * np.cos(pitch_rad / 2) * np.cos(yaw_rad / 2) + np.sin(roll_rad / 2) * np.sin(
        pitch_rad / 2) * np.sin(yaw_rad / 2)

    # 构造旋转矩阵
    rotation_matrix = np.array([[1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                                [2 * (qx * qy + qz * qw), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qx * qw)],
                                [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx ** 2 + qy ** 2)]])

    return np.array(rotation_matrix)


# In[9]:


def calculate_yaw_pitch(vector1, vector2):
    # 标准化向量
    vector1 = vector1 / np.linalg.norm(vector1)
    vector2 = vector2 / np.linalg.norm(vector2)

    # 计算偏转角（yaw）
    yaw = np.arctan2(vector2[1], vector2[0]) - np.arctan2(vector1[1], vector1[0])

    # 计算俯仰角（pitch）
    pitch = np.arcsin(vector2[2]) - np.arcsin(vector1[2])

    # 将弧度转换为角度
    yaw = np.degrees(yaw)
    pitch = np.degrees(pitch)

    return yaw, pitch


# In[10]:


def fit_plane(points):
    # 计算质心
    centroid = np.mean(points, axis=0)

    # 将所有点减去质心
    centered_points = points - centroid

    # 构建增广矩阵 A
    augmented_matrix = np.hstack((centered_points, np.ones((centered_points.shape[0], 1))))

    # 使用最小二乘法求解方程 Ax = b
    _, _, V = np.linalg.svd(augmented_matrix, full_matrices=False)
    plane_params = V[-1, :]

    # 归一化平面法向量
    plane_normal = plane_params[:3] / np.linalg.norm(plane_params[:3])

    # 平面常数项
    plane_constant = plane_params[3]

    # 构建平面参数
    plane = {
        'centroid': centroid,
        'normal': plane_normal,
        'constant': plane_constant
    }

    return plane


# In[11]:

#错误函数
# def calculate_pitch_and_yaw(vector_A, vector_B):
#     # 计算俯仰角
#     pitch_angle = math.acos(vector_A[2] / math.sqrt(vector_A[0] ** 2 + vector_A[1] ** 2 + vector_A[2] ** 2))
#
#     # 计算向量 A 在 XY 平面上的投影
#     vector_A_xy = [vector_A[0], vector_A[1], 0]
#
#     # 计算向量 B 在 XY 平面上的投影
#     vector_B_xy = [vector_B[0], vector_B[1], 0]
#
#     # 计算偏移角
#     cross_product = [vector_A_xy[1] * vector_B_xy[2] - vector_A_xy[2] * vector_B_xy[1],
#                      vector_A_xy[2] * vector_B_xy[0] - vector_A_xy[0] * vector_B_xy[2],
#                      vector_A_xy[0] * vector_B_xy[1] - vector_A_xy[1] * vector_B_xy[0]]
#     dot_product = vector_A_xy[0] * vector_B_xy[0] + vector_A_xy[1] * vector_B_xy[1]
#     yaw_angle = math.atan2(cross_product[2], dot_product)
#
#     # 将弧度转换为度数
#     pitch_angle_degrees = math.degrees(pitch_angle)
#     yaw_angle_degrees = math.degrees(yaw_angle)
#
#     return pitch_angle_degrees, yaw_angle_degrees


# In[12]:


def ray_triangle_intersection(ray_origin, ray_direction, triangle_vertices):
    # 获取三角形的顶点
    vertex0, vertex1, vertex2 = triangle_vertices

    # 计算三角形的法线向量
    triangle_normal = np.cross(vertex1 - vertex0, vertex2 - vertex0)

    # 计算射线与三角形平面的交点
    t = -np.dot(ray_origin - vertex0, triangle_normal) / np.dot(ray_direction, triangle_normal)
    if t < 0:
        return None
    # 计算相交点的坐标
    intersection_point = ray_origin + t * ray_direction

    # 判断相交点是否在三角形内部
    edge0 = vertex1 - vertex0
    edge1 = vertex2 - vertex1
    edge2 = vertex0 - vertex2

    c0 = np.cross(edge0, intersection_point - vertex0)
    c1 = np.cross(edge1, intersection_point - vertex1)
    c2 = np.cross(edge2, intersection_point - vertex2)

    if np.dot(c0, triangle_normal) >= 0 and np.dot(c1, triangle_normal) >= 0 and np.dot(c2, triangle_normal) >= 0:
        return intersection_point
    else:
        return None


# In[13]:


def minimum_bounding_sphere(triangles):
    num_triangles = triangles.shape[0]
    bounding_spheres = [None] * num_triangles

    centers = np.mean(triangles, axis=1)
    subtracted_triangles = triangles - centers[:, np.newaxis, :]
    distances = np.sqrt(np.sum(subtracted_triangles * subtracted_triangles, axis=2))
    max_distance = np.max(distances, axis=1)
    bounding_spheres = {
        'center': centers,
        'radius': max_distance
    }
    return bounding_spheres


# In[14]:


def interacted_triang(point, normal, center_point, vertices, triangles, spheres):
    # 向量的单位方向向量

    normal = normal / np.linalg.norm(normal)

    # 计算点到直线的距离
    differences = spheres['center'] - point
    perpendicular_distances = np.linalg.norm(np.cross(differences, normal), axis=1)
    distances = perpendicular_distances - spheres['radius']

    # 找到最短距离对应的索引
    distances = np.array(distances)
    interacted_index = np.where(distances < 0)

    min_distance = 100
    reserved_point = []
    reserved_normal = []
    reserved_index = None
    for index in range(len(interacted_index[0])):

        distance = 0
        interacted_point = ray_triangle_intersection(point, normal, vertices[triangles[interacted_index[0][index]]])
        if interacted_point is not None:
            distance = np.linalg.norm(interacted_point - center_point)
            if distance < min_distance:
                reserved_point = interacted_point
                reserved_index = interacted_index[0][index]
                min_distance = distance
    if reserved_index is not None:
        edge1 = vertices[triangles[reserved_index]][0] - vertices[triangles[reserved_index]][1]
        edge2 = vertices[triangles[reserved_index]][1] - vertices[triangles[reserved_index]][2]
        tra_normal = np.cross(edge1, edge2)
        reserved_normal = tra_normal / np.linalg.norm(tra_normal)
        if np.dot(normal, reserved_normal) > 0:
            reserved_normal = -reserved_normal
    return reserved_point, reserved_normal

#
# # In[ ]:
#
#
# # In[15]:
#
#
# # 投影仪初始化
# # 输入 unity中 fov
# input_aspect = 9 / 16
# ratio = 2.92
# tan_value = 0.5 * input_aspect / ratio  # 正切值
# # 计算正切值对应的角度
# angle = math.degrees(math.atan(tan_value)) * 2
# print("输入到unity projector fov", angle)
# # proj_tand = math.tan(input_angle * (math.pi / 180))
# # proj_tand = angle_rad/2
#
# # 投影仪初始化方向默认以[0, 1, 0]为起始方向，便于计算，影响平面的生成
# def_projpos = np.array([0, 0, 0])
# def_projvec = np.array([0, 1, 0])
# proj_width = 0.128
# proj_height = 0.072
# proj_len = proj_width * ratio
# proj_plane = np.array([[-proj_width / 2, 0, proj_height / 2],
#                        [-proj_width / 2, 0, -proj_height / 2],
#                        [proj_width / 2, 0, -proj_height / 2],
#                        [proj_width / 2, 0, proj_height / 2]])
# # 采样点数
#
# # 另一种采样，输入采样
# sample_window = [[0, 0],
#                  [1, 0],
#                  [0, 1],
#                  [1, 1]]
#
# # 投影点
#
#
# sampled_matrix = np.zeros_like(sample_window)
#
# sampled_matrix = [[proj_width * item[0] - proj_width / 2, 0, proj_height * (1 - item[1]) - proj_height / 2] for item in
#                   sample_window]
# sampled_matrix = np.array(sampled_matrix)
#
# sampled_proj = sampled_matrix
# print(sampled_matrix)
# # 均匀采样矩形点
#
# # 投影仪移动，按unity输入一致
# proj_x = -0.707
# proj_y = 1
# proj_z = -0.707
# pangle_x = 0  # 绕X轴旋转的角度
# pangle_y = 45  # 绕Y轴旋转的角度
# pangle_z = 0  # 绕Z轴旋转的角度
# rotation_matrix = rotation_angle(-pangle_x, pangle_z, -pangle_y)  # 旋转矩阵 因为unityx轴与输入相反
# proj_normal = np.dot(rotation_matrix, def_projvec)  # 相机方向
# proj_point = (proj_x, proj_z, proj_y)  # 相机起点
# # 旋转相机
# Rimage = transform_cam(proj_point, proj_normal, rotation_matrix, sampled_proj, proj_len)  # Rimage是相机坐标系中的像素点空间坐标
#
# # In[ ]:
#
#
# # In[16]:
#
#
# interactions = [([-0.17123288, 0., 0.90368151]),
#                 ([0.17123288, 0., 0.90368151]),
#                 ([-0.17123288, 0., 1.09631849]),
#                 ([0.17123288, 0., 1.09631849])]
#
# # In[17]:
#
#
# ##############################################################
# # 输出到图像上,图像控制点
# # 定义网格的行数和列数
# num_rows = 5
# num_cols = 5
#
# # 生成行方向和列方向的均匀网格
# x, y = np.meshgrid(np.linspace(0, 1, num_rows), np.linspace(0, 1, num_cols))
#
# # 将网格点坐标组合为二维数组
# all_gridpoints = np.column_stack((1 - x.flatten(), y.flatten()))
#
# # In[19]:
#
#
# # 手动输入部分
# cam_x = 0
# cam_y = 1
# cam_z = -1
# cangle_x = 0  # 绕X轴旋转的角度
# cangle_y = 0  # 绕Y轴旋转的角度
# cangle_z = 0  # 绕Z轴旋转的角度
#
# cam_pos = (cam_x, cam_z, cam_y)
# #cam_normal = np.dot(rotation_matrix2, def_camvec)  # 投影仪方向
#
# # In[20]:
#
#
# # 可视化点云
# fig = plt.figure()
#
# ax = fig.add_subplot(111, projection='3d')
#
# # 绘制相机点云
# ax.scatter(Rimage[:, 0], Rimage[:, 1], Rimage[:, 2], c='r', marker='o')
#
# # 绘制焦点点云
# plot_int = np.array(interactions)
# ax.scatter(plot_int[:, 0], plot_int[:, 1], plot_int[:, 2], c='r', s=10, marker='o')
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
#
# # 设置坐标轴范围一致
# all_point = np.concatenate((Rimage, interactions), axis=0)
# max_range = np.max(all_point, axis=0)
# min_range = np.min(all_point, axis=0)
# ax.set_xlim([min_range[0], max_range[0]])
# ax.set_ylim([min_range[1], max_range[1]])
# ax.set_zlim([min_range[2], max_range[2]])
#
# # 设置坐标轴刻度一致
# x_scale = max_range[0] - min_range[0]
# y_scale = max_range[1] - min_range[1]
# z_scale = max_range[2] - min_range[2]
# max_scale = max([x_scale, y_scale, z_scale])
# ax.set_xlim([min_range[0], min_range[0] + max_scale])
# ax.set_ylim([min_range[1], min_range[1] + max_scale])
# ax.set_zlim([min_range[2], min_range[2] + max_scale])
#
# # In[21]:
#
#
# print(interactions)
#
# # In[22]:
#
#
# ##################################################################################
#
#
# # In[23]:
#
#
# # 进行相机面的计算
# rotation_matrix2 = rotation_angle(-pangle_x, pangle_z, -pangle_y)  # 旋转矩阵 因为unityx轴与输入相反
# rotation_matrix3 = rotation_angle(0, pangle_z, pangle_y)
# rotation_matrix3 = np.dot(rotation_angle(pangle_x, pangle_z, 0), rotation_matrix3)  # 特别注意，由于是倒着旋转，要先转z再转x
#
# proj_pos = np.array((proj_x, proj_z, proj_y))
# proj_normal = np.dot(rotation_matrix2, def_projvec)  # 投影仪方向
# # 生成投影图像平面
# proj_point = []
# projplane_point = proj_pos + np.array(proj_normal) * proj_len
#
# # 计算表面点到投影仪的交点
# for point in interactions:
#     # 以一个点为起点的向量
#
#     start_point = proj_pos
#     vector = (np.array(start_point - point))
#     vector = vector / np.linalg.norm(vector)
#     # 计算向量与平面的交点
#     t = np.dot(proj_normal, projplane_point - start_point) / np.dot(proj_normal, vector)
#     proj_point.append(start_point + t * vector)
#
# # 将平面图像转化为像素,通过旋转到原点的方式，目前先考虑水平，垂直的投影0
#
# proj_point = np.array(proj_point)
# Rproj_plane = transform_proj(proj_pos, proj_normal, rotation_matrix3, proj_point, proj_len)
# proj_points1 = Rproj_plane[:, [0, 2]]
# proj_rect = proj_plane[:, [0, 2]]
#
# # 图像像素到投影面
# proj_points = proj_points1 - proj_rect[1]
# proj_points[:, 0] /= proj_width  # 将第一列元素除以a
# proj_points[:, 1] /= proj_height  # 将第二列元素除以b
# proj_points[:, 1] = 1 - proj_points[:, 1]
# print(proj_points)
#
# # In[ ]:
#
#
# # 读取原始图像
# input_image = cv2.imread('figure1-2.png')
#
# # 获取图像的宽度和高度
# height, width, _ = input_image.shape
#
# # 定义相对位置的四个顶点坐标
# relative_points = np.float32([[0, 1], [1, 1], [0, 0], [1, 0]])
#
# # 将相对位置乘以图像的宽度和高度来得到实际图像中的坐标
# original_points = relative_points * np.float32([width, height])
#
# # 定义目标图像中的四个顶点坐标
# target_points = np.float32(proj_points)
# target_points = target_points * np.float32([width, height])
# # 计算透视变换矩阵
# perspective_matrix = cv2.getPerspectiveTransform(original_points, target_points)
#
# # 进行透视变换
# cam_image = cv2.warpPerspective(input_image, perspective_matrix, (width, height))
#
# cv2.imwrite('output_image.jpg', cam_image)
#
#
# # In[ ]:
#
#
# def show_warped(img, warped):
#     fig, axs = plt.subplots(1, 2, figsize=(16, 8))
#     axs[0].axis('off')
#     axs[1].axis('off')
#     axs[0].imshow(img[..., ::-1], origin='upper')
#     axs[0].scatter(c_src[:, 0] * img.shape[1], c_src[:, 1] * img.shape[0], marker='+', color='red')
#     axs[1].imshow(warped[..., ::-1], origin='upper')
#     axs[1].scatter(c_dst[:, 0] * warped.shape[1], c_dst[:, 1] * warped.shape[0], marker='+', color='red')
#     plt.show()
#
#
# # In[ ]:
#
#
# def warp_image_cv(img, c_src, c_dst, dshape=None):
#     dshape = dshape or img.shape
#     theta = tps.tps_theta_from_points(c_src, c_dst, reduced=True)
#     grid = tps.tps_grid(theta, c_dst, dshape)
#     mapx, mapy = tps.tps_grid_to_remap(grid, img.shape)
#     return cv2.remap(img, mapx, mapy, cv2.INTER_CUBIC)
#
#
# # In[ ]:
#
#
# import cv2
#
# img = cv2.imread('figure1-2.png')
#
# c_src = np.array(sample_window)
# print(c_src)
# c_dst = proj_points
# print(c_dst)
#
# # img = cv2.flip(img, 0)
# # img = cv2.flip(img, 1)
#
#
# warped = warp_image_cv(img, c_src, c_dst, dshape=(720, 1280))
# # warped = cv2.flip(warped, 0)
# # warped = cv2.flip(warped, 1)
# show_warped(img, warped)
# cv2.imwrite('output_image.jpg', warped)
#
# # In[ ]:
#
#
# # 读取原始图像
# input_image = cv2.imread('output_image3.jpg')
#
# # 获取图像的宽度和高度
# height, width, _ = input_image.shape
#
# # 定义相对位置的四个顶点坐标
# relative_points = np.float32(sample_window)
#
# # 将相对位置乘以图像的宽度和高度来得到实际图像中的坐标
# original_points = relative_points * np.float32([width, height])
#
# # 定义目标图像中的四个顶点坐标
# target_points = np.float32(cam_points)
# target_points = target_points * np.float32([width, height])
# print(original_points, target_points)
# # 计算透视变换矩阵
# perspective_matrix = cv2.getPerspectiveTransform(original_points, target_points)
#
# control_points = np.float32([[0.39, 0.45], [0.65, 0.45], [0.39, 0.15], [0.65, 0.15]])
# control_points = control_points * np.float32([width, height])
# perspective_matrix1 = cv2.getPerspectiveTransform(original_points, control_points)
# perspective_matrix2 = cv2.getPerspectiveTransform(target_points, original_points)
#
# transformed_points = cv2.perspectiveTransform(control_points.reshape(-1, 1, 2), perspective_matrix2)
#
# perspective_matrix3 = cv2.getPerspectiveTransform(control_points, transformed_points)
# cam_image2 = cv2.warpPerspective(input_image, perspective_matrix1, (width, height))
# cam_image3 = cv2.warpPerspective(cam_image2, perspective_matrix3, (width, height))
#
# print(target_points, control_points, perspective_matrix2, perspective_matrix)
#
# # 进行透视变换
# cam_image = cv2.warpPerspective(input_image, perspective_matrix, (width, height))
#
# white_image = np.ones((height, width), dtype=np.uint8) * 255
# reserve_area = cv2.warpPerspective(white_image, perspective_matrix, (width, height))
# cv2.imwrite('output_image.jpg', cam_image)
# cv2.imwrite('output_image2.jpg', cam_image2)
# cv2.imwrite('output_image3.jpg', cam_image3)
# cv2.imwrite('output_image4.jpg', reserve_area)
#
# # In[ ]:
#
#
# print(cam_points)
#
# # In[ ]:
#
#
# # 评估算法
#
# eye_degrees = 5  # 角度值
# cam_degrees = 45
# # 将角度转换为弧度
# eye_radians = math.radians(eye_degrees)
# cam_radians = math.radians(cam_degrees)
# # 计算角度的正切值
# eye_tanv = math.tan(eye_radians)
# cam_tanv = math.tan(cam_radians)
# raidus = eye_tanv / cam_tanv
# print(raidus)
#
# # 指标1 视野占比
# center_x = width // 2
# center_y = height // 2
#
# background_image = np.zeros((height, width), dtype=np.uint8)
# cv2.circle(background_image, (center_x, center_y), round(raidus * width), (255), -1)
# back_pixels = np.sum(background_image == 255)
# # 创建和原始图片大小相同的黑色画布
# mask = np.zeros((height, width), dtype=np.uint8)
# # 计算中心坐标
#
# # 在画布上绘制圆形区域
# cv2.circle(mask, (center_x, center_y), round(raidus * width), (255), -1)
# # 将原始图片和画布进行按位与操作，保留圆形区域
# vis_image = cv2.bitwise_and(reserve_area, reserve_area, mask=mask)
# reserve_pixels = np.sum(vis_image == 255)
# ratio1 = reserve_pixels / back_pixels
# # 指标2 可视性程度
# whole_pixels = np.sum(reserve_area == 255)
# ratio2 = reserve_pixels / whole_pixels
# print(ratio1)
# print(ratio2)
# # 显示结果
# cv2.imshow("Result", reserve_area)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
#
# # ###
#
# # In[ ]:
#
#
# # In[ ]:
#
#
# # 可视化点云
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
#
# # 绘制直线
# line_x = [proj_point[0], proj_point[0] + proj_normal[0] * 10]
# line_y = [proj_point[1], proj_point[1] + proj_normal[1] * 10]
# line_z = [proj_point[2], proj_point[2] + proj_normal[2] * 10]
# ax.plot(line_x, line_y, line_z, color='r', label='Line')
#
# # 绘制点云
# ax.scatter(object_cloud[:, 0], object_cloud[:, 1], object_cloud[:, 2], s=10, c='b', marker='.')
#
# # 绘制相机点云
# ax.scatter(Rimage[:, 0], Rimage[:, 1], Rimage[:, 2], c='r', marker='o')
#
# # 绘制焦点点云
# plot_int = np.array(interactions)
# ax.scatter(plot_int[:, 0], plot_int[:, 1], plot_int[:, 2], c='r', s=10, marker='o')
# # 绘制投影仪点云
# ax.scatter(cam_pos[0], cam_pos[1], cam_pos[2], c='g', marker='o')
# line_x2 = [cam_pos[0], cam_pos[0] + cam_normal[0] * 10]
# line_y2 = [cam_pos[1], cam_pos[1] + cam_normal[1] * 10]
# line_z2 = [cam_pos[2], cam_pos[2] + cam_normal[2] * 10]
# ax.plot(line_x2, line_y2, line_z2, color='g', label='Line')
#
# # 设置坐标轴范围一致
# all_point = np.concatenate((Rimage, object_cloud), axis=0)
# max_range = np.max(all_point, axis=0)
# min_range = np.min(all_point, axis=0)
# ax.set_xlim([min_range[0], max_range[0]])
# ax.set_ylim([min_range[1], max_range[1]])
# ax.set_zlim([min_range[2], max_range[2]])
#
# # 设置坐标轴刻度一致
# x_scale = max_range[0] - min_range[0]
# y_scale = max_range[1] - min_range[1]
# z_scale = max_range[2] - min_range[2]
# max_scale = max([x_scale, y_scale, z_scale])
# ax.set_xlim([min_range[0], min_range[0] + max_scale])
# ax.set_ylim([min_range[1], min_range[1] + max_scale])
# ax.set_zlim([min_range[2], min_range[2] + max_scale])
#
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
#




