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