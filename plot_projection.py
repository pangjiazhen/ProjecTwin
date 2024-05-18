import numpy as np
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
robot = rtb.models.URDF.UR5()
q = np.array([1, 2, 3, 4, 5, 6])
T = robot.fkine(q)