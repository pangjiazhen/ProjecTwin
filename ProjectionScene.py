import numpy as np
import ProjectionFun as fp


class Entity:
    def __init__(self):
        self.position = (0, 0, 0)
        self.orientation = (0, 0, 0)
        self.rotationmatrix = []
        self.inrotationmatrix = []
    def move(self, dx, dy, dz):

        self.position = (dx, dy, dz)

    def rotate(self, droll, dpitch, dyaw):

        self.orientation = ( droll,  dpitch,  dyaw)
        self.rotationmatrix = fp.rotation_angle(-droll, dyaw, -dpitch)
        temp_r = fp.rotation_angle(0, dyaw, dpitch)
        self.inrotationmatrix = np.dot(fp.rotation_angle(droll, dyaw, 0), temp_r)  # 特别注意，由于是倒着旋转，要先转z再转x



    def get_position(self):
        return self.position

    def get_orientation(self):
        return self.orientation


class Projector(Entity):
    def __init__(self, aspect, ratio, vector, width, height):
        """
        # set aspect = 9 / 16, ratio = 2.92, vector [0, 1, 0], width = 0.128, height = 0.072
        :type ratio: object
        """
        self.position = (0, 0, 0)
        self.orientation = (0, 0, 0)
        self.aspect = aspect
        self.ratio = ratio
        self.vector = np.array(vector)
        self.width = width
        self.height = height
        self.len = self.width * self.ratio

        self.plane = np.array([[width / 2, 0, -height / 2],
                               [-width / 2, 0, -height / 2],
                               [width / 2, 0, height / 2],
                               [-width / 2, 0, height / 2]])
        num_samples = (2, 2)
        proj_corner = [-width / 2, 0, -height / 2]
        #self.plane = fp.sample_rectangular_points(proj_corner, height, width, num_samples)
        self.stasticplane = self.plane
        self.received = []
        self.planecenter = []

    def transform(self, position):
        x = position[0]
        y = position[1]
        z = position[2]
        roll = position[3]
        pitch = position[4]
        yaw = position[5]
        self.move(x, y, z)
        self.rotate(roll, pitch, yaw)
        self.Nvector = np.dot(self.rotationmatrix, self.vector)
        self.plane = fp.transform_cam(self.position, self.Nvector, self.rotationmatrix, self.plane, self.len)
        self.planecenter = self.position + np.array(self.Nvector) * self.len

    def receive(self,interactions):
        proj_point = []
        for point in interactions:
            # 以一个点为起点的向量
            vector = (np.array(np.array(self.position) - point))
            vector = vector / np.linalg.norm(vector)
            # 计算向量与平面的交点
            t = np.dot(self.Nvector, self.planecenter - self.position) / np.dot(self.Nvector, vector)
            proj_point.append(self.position + t * vector)
        proj_point = np.array(proj_point)

        received_plane = fp.transform_proj(self.position, self.Nvector, self.inrotationmatrix, proj_point, self.len)
        proj_points1 = received_plane[:, [0, 2]]
        proj_rect = self.stasticplane[:, [0, 2]]
        self.test = proj_points1
        proj_points = proj_points1 - proj_rect[1]
        proj_points[:, 0] /= self.width  # 将第一列元素除以a
        proj_points[:, 1] /= self.height  # 将第二列元素除以b
        proj_points[:, 1] = 1 - proj_points[:, 1]
        self.received = proj_points

    def evaluate(self):
        arr = self.received
        if np.any((arr[:, 0] > 1) | (arr[:, 0] < 0) | (arr[:, 1] > 1) | (arr[:, 1] < 0)):
            return 0
        else:
            return fp.calculate_max_convex_hull_area(self.received)

class Object(Entity):
    def __init__(self, path):
        vertices, normals, triangles = fp.read_obj_file(path)
        read_point = np.array(vertices)
        object_cloud = (-read_point[:, 0], read_point[:, 2], read_point[:, 1])
        object_cloud = np.array(object_cloud).T
        # 对于减速器是要乘以10
        self.pointcloud = object_cloud
        self.triangles = triangles = np.array(triangles) - 1


class Observer(Entity):
    def empty(self):
        {}


