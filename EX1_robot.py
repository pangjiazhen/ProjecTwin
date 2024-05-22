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
from matplotlib.patches import Circle
# 场景初始化

# projector初始化，投影仪初始化方向默认以[0, 1, 0]为起始方向，便于计算，影响平面的生成
# set aspect = 9 / 16, ratio = 2.92, vector [0, 1, 0], width = 0.128, height = 0.072
projector = scene.Projector(aspect=9 / 16, ratio=1.37, vector = [0, 1, 0], width= 0.128, height=0.072)
projector.transform([0,0,0,0,0,0])
# observer 初始化
observer = scene.Observer()
cobot = rc.Moblierobot()
temp_p = [1,-2.5, 0.7, -15, -15, 0]
move, joint, jscore, maxcore, ans = cobot.s3_movetrans(temp_p)
print (move, joint, jscore, maxcore, ans)
outjoint = joint*180/3.14
print (move)
print ('joint for unity:',-(outjoint[0]+180),outjoint[1]+90,outjoint[2],outjoint[3]+90,-outjoint[4],outjoint[5])
print ('position for unity:',temp_p[0]-move[0],temp_p[1]-move[1],temp_p[2],0,temp_p[4],0)