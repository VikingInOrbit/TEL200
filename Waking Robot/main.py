from roboticstoolbox import *
import pickle
import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3
from spatialgeometry import Cuboid
from math import pi
from roboticstoolbox.backends.PyPlot import PyPlot

house = rtb_load_matfile("data/house.mat")

floorplan = house["floorplan"]
places = house["places"]

prm = PRMPlanner(occgrid=floorplan, seed=0)

prm.plan(300)
path = prm.query(start=places.br1, goal=places.br2)
def followPath(path):
    for i in range(2, len(path)):
        x1, y1 = path[i-2]
        x2, y2 = path[i-1]
        x3, y3 = path[i]
        v1x = x2 - x1
        v2x = x3 - x2
        v1y = y2 - y1
        v2y = y3 - y2

        lenv1 = np.sqrt(v1x**2 + v1y**2)
        lenv2 = np.sqrt(v2x**2 + v2y**2)

        dot = v1x * v2x + v1y * v2y
        AngleTemp = dot / (lenv1 * lenv2)
        AngleRad =np.arccos(AngleTemp)
        Angle = np.degrees(AngleRad)

        Angle = np.round(Angle).astype(int)
        lenv2 = np.round(lenv2).astype(int)

        for _ in range(int(Angle)):
            pass #Activate function for turning one degree

        for _ in range(int(lenv2)):
            pass #Activate function for walking forward 10cm