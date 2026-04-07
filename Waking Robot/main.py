from roboticstoolbox import ERobot, ET, mstraj
import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3
from spatialgeometry import Cuboid
from math import pi
from roboticstoolbox.backends.PyPlot import PyPlot
import scipy.io
def load_map():
    """
    Loads the map from KillianMap.mat and adds it to the environment.
    You can expand this function to visualize the map as needed.
    """
    try:
        mat = scipy.io.loadmat('../KillianMap.mat')
        print("Map keys:", mat.keys())
        # Try to display a 2D map if present
        mapdata = None
        for key in mat:
            if isinstance(mat[key], np.ndarray) and mat[key].ndim == 2:
                mapdata = mat[key]
                print(f"Displaying map from key: {key}, shape: {mapdata.shape}")
                break
        if mapdata is not None:
            plt.figure("Map Display")
            plt.imshow(mapdata, cmap='gray')
            plt.title(f"Map: {key}")
            plt.colorbar()
            plt.show(block=False)
        else:
            print("No 2D map array found in KillianMap.mat.")
    except Exception as e:
        print(f"Could not load map: {e}")


def start_robot():
    """
    Initializes the robot, environment, and loads the map.
    """
    load_map()
    global mm, L1, L2, W, L, leg, legs, xf, xb, y, zu, zd, segments, x, xcycle, sol, qcycle
    global env, leg_adjustment, body
    print("Starting robot simulation...")
    mm = 0.001   
    L1 = 100 * mm
    L2 = 100 * mm
    W = 100 * mm
    L = 200 * mm

    leg = ERobot(ET.Rz() * ET.Rx() * ET.ty(-L1) * ET.Rx() * ET.tz(-L2))
    legs = [ERobot(leg, name=f'leg{i}') for i in range(4)]

    xf = 50; xb = -xf
    y = -50
    zu = -20; zd = -50
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
    sol = leg.ikine_LM(SE3(xcycle), mask=[1, 1, 1, 0, 0, 0])
    qcycle = sol.q

    env = PyPlot()
    env.launch(limits=[-L, L, -W, W, -0.15, 0.05])

    leg_adjustment = SE3.Rz(pi)
    legs[0].base = SE3(L / 2,  -W / 2, 0)
    legs[1].base = SE3(-L / 2,  -W / 2, 0)
    legs[2].base = SE3(L / 2, W / 2, 0) * leg_adjustment
    legs[3].base = SE3(-L / 2, W / 2, 0) * leg_adjustment

    for leg in legs:
        leg.q = np.r_[0, 0, 0]
        env.add(leg, readonly=True, jointaxes=False, eeframe=False, shadow=False)

    body = Cuboid([L, W, 30 * mm], color='b')
    body.base = SE3(0, 0, 0)
    env.add(body)
    env.step()
      
def stopRobot():
    """Stops the robot simulation and closes all plots."""
    print("Stopping robot simulation...")
    plt.close('all')


def turn(angle, speed):
    """
    Rotates the robot body in place by the given angle (radians) at the given speed (rad/s).
    """
    total_time = abs(angle) / speed
    dt = 0.02
    steps = int(total_time / dt)
    start_theta = body.base.rpy()[2]
    end_theta = start_theta + angle
    theta_positions = np.linspace(start_theta, end_theta, steps)
    for i in range(steps):
        body.base = SE3.Rz(theta_positions[i]) * SE3(body.base.t)
        legs[0].base = body.base * SE3(L / 2,  -W / 2, 0)
        legs[1].base = body.base * SE3( -L / 2,  -W / 2, 0)
        legs[2].base = body.base * SE3(L / 2, W / 2, 0) * leg_adjustment
        legs[3].base = body.base * SE3( -L / 2, W / 2, 0) * leg_adjustment
        animate(i)
        env.step(dt=dt)

    
def goForward(distance, speed):
    """
    Moves the robot body forward by the given distance (meters) at the given speed (m/s).
    """
    total_time = distance / speed
    dt = 0.02
    steps = int(total_time / dt)
    start_x = body.base.t[0]
    end_x = start_x + distance
    x_positions = np.linspace(start_x, end_x, steps)
    for i in range(steps):
        body.base = SE3(x_positions[i], 0, 0)
        legs[0].base = body.base * SE3(L / 2,  -W / 2, 0)
        legs[1].base = body.base * SE3( -L / 2,  -W / 2, 0)
        legs[2].base = body.base * SE3(L / 2, W / 2, 0) * leg_adjustment
        legs[3].base = body.base * SE3( -L / 2, W / 2, 0) * leg_adjustment
        animate(i)
        env.step(dt=dt)

def holdPosition(duration):
    """
    Holds the robot's current position for a given duration (seconds).
    """
    dt = 0.02
    steps = int(duration / dt)
    for i in range(steps):
        env.step(dt=dt)

def animate(i):
    """
    Animates the robot's legs for a given step index.
    """
    legs[0].q = gait(qcycle, i, 0, False)
    legs[1].q = gait(qcycle, i, 100, False)
    legs[2].q = gait(qcycle, i, 200, True)
    legs[3].q = gait(qcycle, i, 300, True)

def main():
    """
    Main simulation loop.
    """
    start_robot()
    goForward(0.5, 0.1)  # Move forward 0.5 meters at 0.1 m/s
    turn(pi/4, 0.1)       # Turn 45 degrees at 0.1 rad/s
    goForward(0.5, 0.1)  # Move forward another 0.5 meters at 0.1 m/s
    holdPosition(2)      # Hold position for 2 seconds
    stopRobot()
#=======================================


def gait(cycle, k, offset, flip):
    """
    Returns the joint angles for a given gait cycle step.
    """
    k = (k + offset) % cycle.shape[0]
    q = cycle[k, :].copy()
    if flip:
        q[0] = -q[0]   # for left-side legs
    return q

if __name__ == "__main__":
    main()