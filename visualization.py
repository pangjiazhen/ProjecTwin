import cv2
import numpy as np
def imcorrection(points,inputpath,outputpath):
    # 读取原始图像
    input_image = cv2.imread(inputpath)

    # 获取图像的宽度和高度
    height, width, _ = input_image.shape

    # 定义相对位置的四个顶点坐标
    relative_points = np.float32([[0, 1], [1, 1], [0, 0], [1, 0]])

    # 将相对位置乘以图像的宽度和高度来得到实际图像中的坐标
    original_points = relative_points * np.float32([width, height])

    # 定义目标图像中的四个顶点坐标
    target_points = np.float32(points)
    target_points = target_points * np.float32([width, height])
    # 计算透视变换矩阵
    perspective_matrix = cv2.getPerspectiveTransform(original_points, target_points)

    # 进行透视变换
    cam_image = cv2.warpPerspective(input_image, perspective_matrix, (width, height))

    cv2.imwrite(outputpath, cam_image)