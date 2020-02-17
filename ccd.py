import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import csv
import math

VIS = True
PAUSETIME = 0.3
mode = "tree"
# read in constraints, ordered by joint index
queries = [[6, -4, -2, 2, .6], [9, -3, 2, 1, .01]]

norm = np.linalg.norm

def normalize(v):
    return v / norm(v)


def quaInv(v):
    return np.array([v[0], -v[1], -v[2], -v[3]])


def toThetaU(source, target):
    if np.array_equal(source, target):
        return (0, source)
    cosTheta = np.dot(source, target)
    u = np.cross(source, target)
    sinTheta = norm(u)
    u /= sinTheta
    theta = math.atan2(sinTheta, cosTheta)
    return (theta, u)


def toQua(source, target):
    (theta, u) = toThetaU(source, target)
    c = math.cos(theta / 2)
    s = math.sin(theta / 2)
    return np.insert(s * u, 0, c)


def quaMult(v1, v2):
    (a, b, c, d) = (v1[0], v1[1], v1[2], v1[3])
    (e, f, g, h) = (v2[0], v2[1], v2[2], v2[3])
    return np.array([a*e-b*f-c*g-d*h, a*f+b*e+c*h-d*g,
                     a*g-b*h+c*e+d*f, a*h+b*g-c*f+d*e])


def quaConj(v1, v2, p):
    if len(p) == 3:
        p = np.insert(p, 0, 0)
    q = toQua(v1, v2)
    qinv = quaInv(q)
    return quaMult(quaMult(q, p), qinv)


def quaConj(q, p):
    if len(p) == 3:
        p = np.insert(p, 0, 0)
    qinv = quaInv(q)
    return quaMult(quaMult(q, p), qinv)


if mode == "list":
    fileName = "list.csv"
elif mode == "tree":
    fileName = "tree.csv"

# read in list of joint positions, ordered from root to end
cr = csv.reader(open(fileName))

if mode == "list":
    joints = np.genfromtxt(fileName, delimiter=',')
    numJoints = len(joints)

    base = joints[0]

    quaternions = np.zeros((numJoints, 4))
    unitVectors = np.zeros((numJoints, 3))
    unitVectors[0][0] = 1
    quaternions[0][0] = 1
    lens = np.zeros(numJoints)
    subSums = np.zeros((numJoints))

    for index in range(1, numJoints):
        diff = joints[index] - joints[index - 1]
        lens[index] = norm(diff)
        unitVectors[index] = diff / lens[index]
        quaternions[index] = toQua(unitVectors[index - 1], unitVectors[index])
        subSums[index] = subSums[index - 1] + lens[index]

    # updates joint positions from start to end exclusive
    def recalc(start, end):
        for index in range(start, end):
            unitVectors[index] = quaConj(
                quaternions[index], unitVectors[index - 1])[1:]
            joints[index] = joints[index - 1] + \
                unitVectors[index] * lens[index]

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax._axis3don = VIS
    fig.show()

    allMet = False
    totalTrials = 0
    while not allMet and totalTrials < 10:
        totalTrials += 1
        allMet = True

        for query in queries:
            endEffInd = query[0]

            if endEffInd == 0:
                print("Base unmovable")
                exit()

            desiredPos = np.array(query[1: -1])
            threshold = query[-1]

            # check whether target is reachable
            if norm(desiredPos - base) > subSums[endEffInd]:
                # draw target
                for q in queries:
                    ax.plot([q[1]], [q[2]], [q[3]], 'r+')

                # reach straight to it
                unitVectors[0] = normalize(desiredPos - base)
                for index in range(1, numJoints):
                    quaternions[index] = np.array([1, 0, 0, 0])
                recalc(1, numJoints)

                for index in range(numJoints - 1):
                    ax.plot([joints[index][0], joints[index + 1][0]],
                            [joints[index][1], joints[index + 1][1]],
                            [joints[index][2], joints[index + 1][2]])

                ax._axis3don = VIS
                fig.canvas.draw()
                print("Unachievable")
                plt.pause(3)
                exit()

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
            while norm(desiredPos - joints[endEffInd]) > threshold and trials < 1000:
                ax.set_title("trial " + str(trials) + " error: " +
                             str(norm(desiredPos - joints[numJoints - 1])))

                trials += 1
                allMet = False

                

                for curInd in range(endEffInd - 1, -1, -1):
                    desiredVec = normalize(desiredPos - joints[curInd])
                    currentVec = normalize(
                        joints[endEffInd] - joints[curInd])

                    quaternions[curInd + 1] = quaMult(toQua(currentVec, desiredVec),
                                                      quaternions[curInd + 1])

                    # update all joints positions
                    recalc(curInd + 1, numJoints)

                for q in queries:
                    ax.plot([q[1]], [q[2]], [q[3]], 'r+')
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

            # print(norm(desiredPos - joints[endEffInd]))

            if trials == 1000:
                print("More than 1000 trials. Please increase your threshold.")
                exit()
            elif trials != 0:
                print("trials: " + str(trials))

    plt.close(fig)
    exit()

elif mode == "tree":
    joints = np.genfromtxt(fileName, delimiter=',')
    numJoints = len(joints)

    base = joints[0][1:]

    quaternions = np.zeros((numJoints, 4))
    unitVectors = np.zeros((numJoints, 3))
    unitVectors[0][0] = 1
    quaternions[0][0] = 1
    lens = np.zeros(numJoints)
    subSums = np.zeros((numJoints))

    for index in range(1, numJoints):
        parent = int(joints[index][0])
        diff = joints[index][1:] - joints[parent][1:]
        lens[index] = norm(diff)
        unitVectors[index] = diff / lens[index]
        quaternions[index] = toQua(unitVectors[parent], unitVectors[index])
        subSums[index] = subSums[parent] + lens[index]

    # updates joint positions from start to end exclusive
    def recalc(start, end):
        for index in range(start, end):
            parent = int(joints[index][0])
            unitVectors[index] = quaConj(
                quaternions[index], unitVectors[parent])[1:]
            joints[index][1:] = joints[parent][1:] + \
                unitVectors[index] * lens[index]

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax._axis3don = VIS
    fig.show()

    allMet = False
    totalTrials = 0
    while not allMet and totalTrials < 10:
        totalTrials += 1
        allMet = True

        for query in queries:
            endEffInd = query[0]

            if endEffInd == 0:
                print("Base unmovable")
                exit()

            desiredPos = np.array(query[1: -1])
            threshold = query[-1]

            # check whether target is reachable
            if norm(desiredPos - base) > subSums[endEffInd]:
                # draw target
                for q in queries:
                    ax.plot([q[1]], [q[2]], [q[3]], 'r+')

                # reach straight to it
                unitVectors[0] = normalize(desiredPos - base)
                for index in range(1, numJoints):
                    quaternions[index] = np.array([1, 0, 0, 0])
                recalc(1, numJoints)

                for index in range(1, numJoints):
                    parent = int(joints[index][0])
                    ax.plot([joints[parent][1], joints[index][1]],
                            [joints[parent][2], joints[index][2]],
                            [joints[parent][3], joints[index][3]])

                ax._axis3don = VIS
                fig.canvas.draw()
                print("Unachievable")
                plt.pause(10)
                exit()

            trials = 0
            while norm(desiredPos - joints[endEffInd][1:]) > threshold and trials < 1000:
                ax.set_title("trial " + str(trials) + " error: " +
                             str(norm(desiredPos - joints[endEffInd][1:])))

                trials += 1
                allMet = False

                for q in queries:
                    ax.plot([q[1]], [q[2]], [q[3]], 'r+')

                # draw lines
                for index in range(1, numJoints):
                    parent = int(joints[index][0])
                    ax.plot([joints[parent][1], joints[index][1]],
                            [joints[parent][2], joints[index][2]],
                            [joints[parent][3], joints[index][3]])

                ax._axis3don = VIS
                ax.set_xlim(-6, 6)
                ax.set_ylim(-6, 6)
                ax.set_zlim(0, 6)
                fig.canvas.draw()
                plt.pause(PAUSETIME)
                ax.clear()

                prevInd = endEffInd
                curInd = int(joints[endEffInd][0])
                while curInd != 0:
                    desiredVec = normalize(desiredPos - joints[curInd][1:])
                    currentVec = normalize(
                        joints[endEffInd][1:] - joints[curInd][1:])

                    quaternions[prevInd] = quaMult(toQua(currentVec, desiredVec),
                                                   quaternions[prevInd])

                    # update all joints positions
                    recalc(1, numJoints)

                    prevInd = curInd
                    curInd = int(joints[curInd][0])

                desiredVec = normalize(desiredPos - joints[curInd][1:])
                currentVec = normalize(
                    joints[endEffInd][1:] - joints[curInd][1:])

                quaternions[prevInd] = quaMult(toQua(currentVec, desiredVec),
                                               quaternions[prevInd])

                # update all joints positions
                recalc(1, numJoints)


            # print(norm(desiredPos - joints[endEffInd]))

            if trials == 1000:
                print("More than 1000 trials. Please increase your threshold.")
                exit()

    plt.close(fig)
    exit()
