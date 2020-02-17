import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import csv
import math

VIS = True
PAUSETIME = .3

norm = np.linalg.norm

def normalize(v):
    return v / norm(v)

fileName = "list.csv"

# read in list of joint positions, ordered from root to end
cr = csv.reader(open(fileName))

joints = np.genfromtxt(fileName, delimiter=',')
numJoints = len(joints)

base = joints[0].copy()

lens = np.zeros(numJoints)

for index in range(1, numJoints):
    diff = joints[index] - joints[index - 1]
    lens[index] = norm(diff)

fig = plt.figure()
ax = fig.gca(projection='3d')
ax._axis3don = VIS
fig.show()

# read in constraints, ordered by joint index
query = [12, 15, 10, .006]

desiredPos = np.array(query[:-1])
threshold = query[-1]

# check whether target is reachable
if norm(desiredPos - base) > np.sum(lens):
    # draw target
    ax.plot([query[0]], [query[1]], [query[2]], 'r+')

    # reach straight to it
    unitVector = normalize(desiredPos - base)
    for index in range(1, numJoints):
        joints[index] = joints[index - 1] + lens[index] * unitVector

    for index in range(numJoints - 1):
        ax.plot([joints[index][0], joints[index + 1][0]],
                [joints[index][1], joints[index + 1][1]],
                [joints[index][2], joints[index + 1][2]])

    ax._axis3don = VIS
    fig.canvas.draw()
    print("Unachievable")
    plt.pause(10)
    exit()

ax.plot([query[0]], [query[1]], [query[2]], 'r+')

# draw lines
for index in range(numJoints - 1):
    ax.plot([joints[index][0], joints[index + 1][0]],
            [joints[index][1], joints[index + 1][1]],
            [joints[index][2], joints[index + 1][2]])

ax._axis3don = VIS
ax.set_xlim(0, 20)
ax.set_ylim(0, 20)
ax.set_zlim(0, 20)
fig.canvas.draw()
plt.pause(PAUSETIME)
ax.clear()

trials = 0
while norm(desiredPos - joints[numJoints - 1]) > threshold and trials < 1000:
    # print(norm(desiredPos - joints[numJoints - 1]))

    trials += 1

    # backward reaching
    end = query[:-1]
    joints[numJoints - 1] = end
    for curInd in range(numJoints - 2, -1, -1):
        direction = normalize(joints[curInd] - end)
        joints[curInd] = end + direction * lens[curInd + 1]
        end = joints[curInd]

    end = base
    joints[0] = end
    for curInd in range(1, numJoints):
        direction = normalize(joints[curInd] - end)
        joints[curInd] = end + direction * lens[curInd]
        end = joints[curInd]
    
    ax.plot([query[0]], [query[1]], [query[2]], 'r+')

    # draw lines
    for index in range(numJoints - 1):
        ax.plot([joints[index][0], joints[index + 1][0]],
                [joints[index][1], joints[index + 1][1]],
                [joints[index][2], joints[index + 1][2]])

    ax._axis3don = VIS
    ax.set_xlim(0, 20)
    ax.set_ylim(0, 20)
    ax.set_zlim(0, 20)
    fig.canvas.draw()
    plt.pause(PAUSETIME)
    ax.clear()

# print(norm(desiredPos - joints[numJoints - 1]))

if trials == 1000:
    print("More than 1000 trials. Please increase your threshold.")
    exit()

plt.close(fig)
exit()
