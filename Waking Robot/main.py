from roboticstoolbox import *
import pickle
import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3
from spatialgeometry import Cuboid
from math import pi
from roboticstoolbox.backends.PyPlot import PyPlot

mm = 0.001   
L1 = 100 * mm
L2 = 100 * mm
W = 100 * mm; L = 200 * mm

house = rtb_load_matfile("data/house.mat")

floorplan = house["floorplan"]
places = house["places"]

prm = PRMPlanner(occgrid=floorplan, seed=0)

prm.plan(300)
path = prm.query(start=places.br1, goal=places.br2)

print
def followPath(path):
    for i in range(1, len(path)):
        x1, y1 = path[i-1]
        x2, y2 = path[i]
        dot = x1 * x2 + y1 * y2
        len1 = np.sqrt(x1**2 + y1**2)
        len2 = np.sqrt(x2**2 + y2**2)
        newAngle = dot / (len1 * len2)
        newAngle = np.arccos()
        print(newAngle)

followPath(path)

"""
leg = ERobot(ET.Rz() * ET.Rx() * ET.ty(-L1) * ET.Rx() * ET.tz(-L2))

legs = [
    ERobot(leg, name='leg0'),
    ERobot(leg, name='leg1'),
    ERobot(leg, name='leg2'),
    ERobot(leg, name='leg3')
]

xf = 50; xb = -xf;   
y = -50;              
zu = -20; zd = -50;     

segments = np.array([
    [xf, y, zd],
    [xb, y, zd],
    [xb, y, zu],
    [xf, y, zu],
    [xf, y, zd]
     ]) * mm

x = mstraj(segments, tsegment=[3, 0.25, 0.5, 0.25], dt=0.01, tacc=0.07)

xcycle = x.q
xcycle = np.vstack((xcycle, xcycle[-3:,:]))

sol = leg.ikine_LM( SE3(xcycle), mask=[1, 1, 1, 0, 0, 0] )


qcycle = sol.q

env = PyPlot()
env.launch(limits=[-L, L, -W, W, -0.15, 0.05])

leg_adjustment = SE3.Rz(pi)
legs[0].base = SE3(L / 2,  -W / 2, 0)
legs[1].base = SE3( -L / 2,  -W / 2, 0)
legs[2].base = SE3(L / 2, W / 2, 0) * leg_adjustment
legs[3].base = SE3( -L / 2, W / 2, 0) * leg_adjustment

for leg in legs:
    leg.q = np.r_[0, 0, 0]
    env.add(leg, readonly=True, jointaxes=False, eeframe=False, shadow=False)


body = Cuboid([L, W, 30 * mm], color='b')
body.base = SE3(0, 0, 0)  
env.add(body)

env.step()

def gait(cycle, k, offset, flip):
    k = (k + offset) % cycle.shape[0]
    q = cycle[k, :].copy()
    if flip:
        q[0] = -q[0]   # for left-side legs
    return q

env.step()


for i in range(4000):
    if not plt.fignum_exists(env.fig.number):
        break

    legs[0].q = gait(qcycle, i, 0, False)
    legs[1].q = gait(qcycle, i, 100, False)
    legs[2].q = gait(qcycle, i, 200, True)
    legs[3].q = gait(qcycle, i, 300, True)  

    env.step(dt=0.02)


env.hold()



plt.close('all')
"""