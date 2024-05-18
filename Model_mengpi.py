import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree



class SamplingMengpi:
    def __init__(self, center, mengpi_radius, angle_start , angle_end , mengpi_height):
        self.center = center
        self.radius = mengpi_radius
        self.angle_start = angle_start
        self.angle_end = angle_end
        self.height = mengpi_height
        self.samplingcloud = []
        self.angles =[]
    def pointsampling(self, angle_step,z_step):

        # 将角度转换为弧度
        angle_start_rad = np.deg2rad(self.angle_start)
        angle_end_rad = np.deg2rad(self.angle_end)
        angle_step_rad = np.deg2rad(angle_step)

        # 生成角度的数组
        angle_set = np.arange(angle_start_rad, angle_end_rad + angle_step_rad, angle_step_rad)

        # 计算每个角度对应的x坐标和y坐标
        x = self.center[0] + self.radius * np.cos(angle_set)
        y = self.center[1] + self.radius * np.sin(angle_set)

        # 生成z的数组
        z = np.arange(0, self.height + z_step, z_step)

        # 生成所有点的坐标
        points = []
        angles = []
        for i in range(len(x)):
            for j in range(len(z)):
                points.append([x[i], y[i], z[j]])
                angles.append(angle_set[i])
        # 将点的坐标转换为NumPy数组
        self.samplingcloud = np.array(points)
        self.angles = angles

    def samgeneration(self, id, x_grids, y_grids, intervallength, intervalangle):
        angle = self.angles[id]
        point = self.samplingcloud[id]
        angle_start_rad = angle - np.deg2rad((x_grids-1)/2 * intervalangle)
        angle_end_rad = angle + np.deg2rad((x_grids-1)/2 * intervalangle)
        angle_step_rad = np.deg2rad(intervalangle)

        # 生成角度的数组
        angles = np.arange(angle_start_rad, angle_end_rad + angle_step_rad-0.01, angle_step_rad)

        # 计算每个角度对应的x坐标和y坐标
        x = self.center[0] + self.radius * np.cos(angles)
        y = self.center[1] + self.radius * np.sin(angles)

        # 生成z的数组
        z = np.arange(point[2]-(y_grids-1)/2*intervallength, point[2]+(y_grids-1)/2*intervallength + intervallength-0.01, intervallength)
        points = []
        for i in range(len(x)):
            for j in range(len(z)):
                points.append([x[i], y[i], z[j]])
        return points

