import math
import random
import numpy as np
import ProjectionScene as ps
import ProjectionFun as fp
import RobotControl as rc

def generate_sphere_point(N):
    points = [[0, 0, 0] for _ in range(N)]
    phi = 0.618
    for n in range(N):
        z = (2 * n - 1) / N - 1
        r = np.sqrt(np.clip(1 - z * z, 0, 1))  # 使用np.clip()限制参数范围
        x = r * np.cos(2 * np.pi * n * phi)
        y = r * np.sin(2 * np.pi * n * phi)
        points[n][0] = x
        points[n][1] = y
        points[n][2] = z
    return np.array(points)

def robot_fesi_restrict(points,hmax,hmin):

    return [point for point in points if ((point[2] <= hmax)and(point[2] >= hmin))]

def cylinder_gen(radius, height):
    surface_area = 2 * math.pi * radius * (radius + height)
    return  surface_area

def block2D_detection(points, pos, radius, center):
    #圆柱体范围
    # pos = np.array(pos)
    points = np.array(points)
    points2D = points[:, [0, 1]]
    pos = np.array(pos)
    pos2D = pos[[0, 1]]
    center = np.array(center)
    center2D = center[[0,1]]
    lines = center2D - points2D
    line_vectors = [-lines[:,1], lines[:,0]]
    line_vectors = np.array(line_vectors).T
    point_vectors = pos2D - points2D
    point_vectors = np.array(point_vectors)
    line_vectors = np.array(line_vectors)
    row_sum = np.sum(point_vectors * line_vectors, axis=1)
    distances = np.abs(row_sum) / np.linalg.norm(line_vectors)

    return points [[distances>radius]]



class Pfield:
    def __init__(self):
        self.pointcloud = []
        self.position = []
        self.block = []
        self.areas = []
    def rt1_range(self, num, p, r):
        points = np.array(generate_sphere_point(num) * r) +p
        points = points[points[:, 1] < p[1]]
        self.pointcloud = points[points[:, 2] > 0]

    def rt2_block(self, type, pos, radius, center):
        if (type == 'cylinder'):
            self.pointcloud = block2D_detection(self.pointcloud, pos, radius, center)

    def rt3_cast(self, ppoints, projector):

        filtered_points = []
        filtered_areas = []
        filtered_positions = []
        for point in self.pointcloud:
            vproj = np.mean(ppoints,axis = 0) - point
            x, y ,z = point
            b, a  = fp.calculate_yaw_pitch(vproj,projector.vector)
            position = [x,y,z,a,b,0]
            projector.transform(position)
            projector.receive(ppoints)
            area = fp.calculate_max_convex_hull_area(projector.received)
            arr = projector.received
            if np.any((arr[:, 0] > 0.9) | (arr[:, 0] < 0.1) | (arr[:, 1] > 0.9) | (arr[:, 1] < 0.1)):
                {}
            else:
                filtered_points.append(point)
                filtered_areas.append(area)
                filtered_positions.append(position)

        combined = list(zip(filtered_areas, filtered_points, filtered_positions))
        if filtered_areas:
            sorted_combined = sorted(combined, key=lambda x: x[0], reverse=True)
            sorted_area, sorted_pointcloud, sorted_positions = zip(*sorted_combined)
            self.pointcloud = sorted_pointcloud
            self.areas = sorted_area
            self.position =sorted_positions
        else:
            self.pointcloud = filtered_points
            self.areas = filtered_areas
            self.position = filtered_positions


    def rt4_robot(self, cobot, type):
        areas = self.areas
        position = self.position
        for a, p in zip(areas,position):
            temp_p = p.copy()
            if type == 'fixed':
                move, joint, jscore, maxcore, ans = cobot.s1_fixed(temp_p)
            if type == 'move':
                move, joint, jscore, maxcore, ans = cobot.s2_move(temp_p)
            if type == 'movetrans':
                move, joint, jscore, maxcore, ans = cobot.s3_movetrans(temp_p)
            if ans == True:
                return a, p, move, joint, jscore, maxcore, ans
        return [], [], [], [], [], [], ans


    def rt5_bestview(self, position):
        {}

    def getposition(self, ppoints, projector):
        position = []
        for point in self.pointcloud:
            vproj = np.mean(ppoints,axis = 0) - point
            x, y ,z = point
            b, a  = fp.calculate_yaw_pitch(vproj,projector.vector)
            position.append([x,y,z,a,b,0])
        self.position = position



