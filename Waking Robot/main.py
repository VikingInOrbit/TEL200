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

        v1 = [x2 - x1, y2 - y1]
        v2 = [x3 - x2, y3 - y2]

        lenv1 = np.sqrt(v1[0]**2 + v1[1]**2)
        lenv2 = np.sqrt(v2[0]**2 + v2[1]**2)
        print(lenv2)
        dot = v1[0] * v2[0] + v1[1] * v2[1]
        AngleTemp = dot / (lenv1 * lenv2)
        AngleRad =np.arccos(AngleTemp)
        Angle = np.degrees(AngleRad)
        print(Angle)

        Angle = np.round(Angle).astype(int)
        lenv2 = np.round(lenv2).astype(int)
        

        print(f"v1: {v1}, v2: {v2}")
        for _ in range(int(Angle)):
            pass #Activate function for turning one degree

        for _ in range(int(lenv2)):
            pass #Activate function for walking forward 10cm

followPath(path)

