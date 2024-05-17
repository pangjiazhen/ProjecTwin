import math
import random
import numpy as np
import projectionscene as scene
import fun_projection as fp


def generate_sphere_point(N):
    points = [[0,0,0] for _ in range(N)]
    phi = 0.618
    for n in range(N):
        z = (2*n-1)/N-1
        x = np.sqrt(1 - z * z) * np.cos(2 * np.pi * n * phi)
        y = np.sqrt(1 - z * z) * np.sin(2 * np.pi * n * phi)
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
    def rt1_range(self, num, p, r):
        points = generate_sphere_point(num) * r + p
        points = points[points[:, 1] < 0]
        self.pointcloud = points[points[:, 2] > 0]

    def rt2_robot(self,hmax,hmin):
        self.pointcloud = robot_fesi_restrict(self.pointcloud,hmax,hmin)

    def rt3_block(self, type, pos, radius, center):
        if (type == 'cylinder'):
            self.pointcloud = block2D_detection(self.pointcloud, pos, radius, center)

    def rt4_cast(self, ppoints, projector ):

        filtered_points = []
        for point in self.pointcloud:
            vproj = np.mean(ppoints,axis = 0) - point
            x, y ,z = point
            b, a  = fp.calculate_yaw_pitch(vproj,projector.vector)
            projector.transform([x,y,z,a,b,0])
            projector.receive(ppoints)
            area = fp.calculate_max_convex_hull_area(projector.received)

            if ((area>0.1) and (np.min(projector.received)>0) and (np.max(projector.received)<1)):
                filtered_points.append(point)
        self.pointcloud = filtered_points


    def rt5_visual(self, position):
        {}

    def getposition(self, ppoints, projector):
        position = []
        for point in self.pointcloud:
            vproj = np.mean(ppoints,axis = 0) - point
            x, y ,z = point
            b, a  = fp.calculate_yaw_pitch(vproj,projector.vector)
            position.append([x,y,z,a,b,0])
        self.position = position



