import copy
import os
from collections import defaultdict
import numpy as np
import matplotlib.pyplot as plt
import math
import motion_model
import ExportJson

# Gradient descent related parameters
learningRate = 0.1
xtol = 0.02
ytol = 0.02
thetaTol = 0.01
kTol = 0.01
max_iter = 100
deltaParams = np.array([0.1, 0.02, 0.02, 0.02])

# Visualization flags
show_animation = False
plot_trajectories = False

# Trajectory parameters
speedDiscretization = 3.0
maxSpeed = 21.0
initialSpeed = 3.0
maxCurvatureRate = 0.20
minX = 10.0
maxX = 50.0
laneSize = 4.0


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              fc=fc, ec=ec, head_width=width, head_length=width)
    plt.plot(x, y)
    plt.plot(0, 0)


def GetJacobian(target, params, deltaParams, state):

    J=[]

    newParam = params
    newParam[0] = newParam[0] + deltaParams[0]
    traj = motion_model.generate_trajectory(newParam, state)
    dp = [(target.x - traj[-1].x), (target.y - traj[-1].y), (target.yaw - traj[-1].yaw), (target.k - traj[-1].k)]

    newParam = params
    newParam[0] = newParam[0] - deltaParams[0]
    traj = motion_model.generate_trajectory(newParam, state)
    dn = [(target.x - traj[-1].x), (target.y - traj[-1].y), (target.yaw - traj[-1].yaw), (target.k - traj[-1].k)]

    dp = np.array(dp)
    dn = np.array(dn)
    J1 = (dp-dn)/(2*deltaParams[0])
    J.append(J1)


    newParam = params
    newParam[1] = newParam[1] + deltaParams[1]
    traj = motion_model.generate_trajectory(newParam, state)
    dp = [(target.x - traj[-1].x), (target.y - traj[-1].y), (target.yaw - traj[-1].yaw), (target.k - traj[-1].k)]

    newParam = params
    newParam[1] = newParam[1] - deltaParams[1]
    traj = motion_model.generate_trajectory(newParam, state)
    dn = [(target.x - traj[-1].x), (target.y - traj[-1].y), (target.yaw - traj[-1].yaw), (target.k - traj[-1].k)]

    dp = np.array(dp)
    dn = np.array(dn)
    J1 = (dp - dn) / (2 * deltaParams[1])
    J.append(J1)



    newParam = params
    newParam[2] = newParam[2] + deltaParams[2]
    traj = motion_model.generate_trajectory(newParam, state)
    dp = [(target.x - traj[-1].x), (target.y - traj[-1].y), (target.yaw - traj[-1].yaw), (target.k - traj[-1].k)]

    newParam = params
    newParam[2] = newParam[2] - deltaParams[2]
    traj = motion_model.generate_trajectory(newParam, state)
    dn = [(target.x - traj[-1].x), (target.y - traj[-1].y), (target.yaw - traj[-1].yaw), (target.k - traj[-1].k)]

    dp = np.array(dp)
    dn = np.array(dn)
    J1 = (dp - dn) / (2 * deltaParams[2])
    J.append(J1)



    newParam = params
    newParam[3] = newParam[3] + deltaParams[3]
    traj = motion_model.generate_trajectory(newParam, state)
    dp = [(target.x - traj[-1].x), (target.y - traj[-1].y), (target.yaw - traj[-1].yaw), (target.k - traj[-1].k)]

    newParam = params
    newParam[3] = newParam[3] - deltaParams[3]
    traj = motion_model.generate_trajectory(newParam, state)
    dn = [(target.x - traj[-1].x), (target.y - traj[-1].y), (target.yaw - traj[-1].yaw), (target.k - traj[-1].k)]

    dp = np.array(dp)
    dn = np.array(dn)
    J1 = (dp - dn) / (2 * deltaParams[3])
    J.append(J1)

    J = np.array(J).T

    return J


def show_trajectory(target, xc, yc):

    plt.clf()
    plot_arrow(target.x, target.y, target.yaw)
    plt.plot(xc, yc, "-r")
    plt.axis("equal")
    plt.grid(True)
    plt.pause(0.1)


def ReachedTarget(target, lastState):

    if math.fabs(target.x-lastState.x) > xtol:
        return False
    elif math.fabs(target.y-lastState.y) > ytol:
        return False
    elif math.fabs(target.yaw-lastState.yaw) > thetaTol:
        return False
    elif math.fabs(target.k-lastState.k) > kTol:
        return False

    return True


def optimizePath(target, state, params):

    p = copy.deepcopy(params)
    foundPath = False

    for i in range(max_iter):
        traj = motion_model.generate_trajectory(p, state)

        if ReachedTarget(target, traj[-1]):
            # print("found path")
            foundPath = True

            # snap the last configuration to target state, to avoid discont
            # during planning
            traj[-1].x = target.x
            traj[-1].y = target.y
            traj[-1].yaw = target.yaw
            traj[-1].k = target.k

            break
        else:

            error = [(target.x-traj[-1].x),(target.y-traj[-1].y),(target.yaw-traj[-1].yaw),(target.k-traj[-1].k)]
            error = np.array(error)
            error = error.reshape(error.shape[0],1)
            J = GetJacobian(target, params, deltaParams, state)

            try:
                dp = - np.dot(np.linalg.inv(J), error)
            except np.linalg.linalg.LinAlgError:
                print("cannot calc path LinAlgError")
                xc, yc, yawc, p = None, None, None, None
                break
            alpha = learningRate

            p += alpha * dp.reshape(dp.shape[0])

            if show_animation:
                x = [traj[i].x for i in range(len(traj))]
                y = [traj[i].y for i in range(len(traj))]
                theta = [traj[i].yaw for i in range(len(traj))]

                show_trajectory(target, x, y)

    return traj, params, foundPath


def test_optimize_path(x=25.0, y=5.0):

    #  target = motion_model.State(x=5.0, y=2.0, yaw=np.deg2rad(00.0))
    target = motion_model.State(x=x, y=y, yaw=np.deg2rad(0.0), s=0.0, k=0.0)
    initial_state = motion_model.State(x=0.0, y=0.0, yaw=np.deg2rad(0.0), s=0.0, k=0.0)

    init_p = np.array([x, 0.0, 0.0, 0.0])

    traj, params, foundPath = optimizePath(target,initial_state, init_p)

    if foundPath:
        x = [traj[i].x for i in range(len(traj))]
        y = [traj[i].y for i in range(len(traj))]
        yaw = [traj[i].yaw for i in range(len(traj))]
        for i in range(len(x)):
            plot_arrow(x[i], y[i], yaw[i], length=0.1)

        # if plot_trajectories:
        #     show_trajectory(target, x, y)
        #     plt.axis("equal")
        #     plt.grid(True)
        #     plt.show()
        return traj
    else:
        return []


def GetTrajectory(path, speed, acc):

    if len(path)==0:
        return []

    traj = []
    newState = motion_model.State(x=path[0].x, y=path[0].y, yaw=path[0].yaw, s=path[0].s, k=path[0].k)
    newState.v = speed
    newState.a = acc
    newState.t = 0
    traj.append(newState)
    lastCurvature = path[0].k

    if acc==-1:
        acc = ((speed-speedDiscretization)**2 - speed**2) / (2*path[-1].s)
    elif acc==1:
        acc = ((speed+speedDiscretization)**2 - speed**2) / (2 * path[-1].s)

    for i in range(1, len(path)):

        state = path[i]

        if speed**2 + 2*acc*state.s<0:
            return []
        else:
            v = np.sqrt(speed**2 + 2*acc*state.s)

        if acc==0.0:
            t = state.s /speed
        else:
            t = (v - speed)/acc

        if v>=initialSpeed and t>0 and v<=maxSpeed:
            newState = motion_model.State(x=state.x, y=state.y, yaw=state.yaw, s=state.s, k=state.k)
            newState.v = v
            newState.a = acc
            newState.t = t

            if np.abs(state.k - lastCurvature)/t>maxCurvatureRate:
                return []
            else:
                traj.append(newState)
        else:
            return []

    return traj

def PlotTraj(traj):
    x = [traj[i].x for i in range(len(traj))]
    y = [traj[i].y for i in range(len(traj))]
    yaw = [traj[i].yaw for i in range(len(traj))]
    for i in range(len(x)):
        plot_arrow(x[i], y[i], yaw[i], length=0.1)

    show_trajectory(traj[-1], x, y)
    plt.axis("equal")
    plt.grid(True)
    plt.show()

def GetPureTrajectory(traj):

    final = []
    for state in traj:
        inTuple = (state.x, -state.y, state.yaw, state.s, state.k, state.v, state.t, state.a)
        final.append(inTuple)
    return final

def GenerateTrajectories():

    accelerations = np.array([-1.0,0.0,1.0])
    speeds = np.linspace(initialSpeed, maxSpeed, num=7)

    xArr = np.linspace(minX,maxX,num=9)
    trajs = defaultdict(list)

    allVariations = np.ones((accelerations.shape[0], speeds.shape[0]))
    for i in range(xArr.shape[0]):

        path = test_optimize_path(x=xArr[i], y=laneSize)

        for j in range(speeds.shape[0]):
            for k in range(accelerations.shape[0]):

                # we have already found the trajectory
                if allVariations[k,j]==0:
                    continue

                traj = GetTrajectory(path, speeds[j], accelerations[k])

                if len(traj)>0:
                    print("Found one at speed", speeds[j],
                          "acc:", accelerations[k],
                          "x:", traj[-1].x,
                          "y:", traj[-1].y,
                          "t:", traj[-1].t,
                          "v:", traj[-1].v)

                    if plot_trajectories:
                        PlotTraj(traj)

                    # just takes data from state objects and puts them as tuples.
                    newTraj = GetPureTrajectory(traj)
                    trajs[speeds[j]].append(newTraj)
                    allVariations[k,j] = 0
                else:
                    continue

                if np.sum(np.sum(allVariations))==0:
                    print("Got all speed, acc variations")


    allVariations = np.ones((accelerations.shape[0], speeds.shape[0]))
    for i in range(xArr.shape[0]):

        path = test_optimize_path(x=xArr[i], y=-laneSize)

        for j in range(speeds.shape[0]):
            for k in range(accelerations.shape[0]):

                # we have already found the trajectory
                if allVariations[k,j]==0:
                    continue

                traj = GetTrajectory(path, speeds[j], accelerations[k])

                if len(traj)>0:
                    print("Found one at speed", speeds[j],
                          "acc:",accelerations[k],
                          "x:", traj[-1].x,
                          "y:", traj[-1].y,
                          "t:", traj[-1].t,
                          "v:", traj[-1].v)

                    if plot_trajectories:
                        PlotTraj(traj)

                    # just takes data from state objects and puts them as tuples.
                    newTraj = GetPureTrajectory(traj)
                    trajs[speeds[j]].append(newTraj)
                    allVariations[k,j] = 0
                else:
                    continue

                if np.sum(np.sum(allVariations))==0:
                    print("Got all speed, acc variations")


    straightPath = test_optimize_path(x=5, y=0)
    for j in range(speeds.shape[0]):
        for k in range(accelerations.shape[0]):

            traj = GetTrajectory(straightPath, speeds[j], accelerations[k])

            if len(traj) > 0:
                print("Found one at speed", speeds[j],
                      "acc:", accelerations[k],
                      "x:", traj[-1].x,
                      "y:", traj[-1].y,
                      "t:", traj[-1].t,
                      "v:", traj[-1].v)

                if plot_trajectories:
                    PlotTraj(traj)

                # just takes data from state objects and puts them as tuples.
                newTraj = GetPureTrajectory(traj)
                trajs[speeds[j]].append(newTraj)
            else:
                continue

    return trajs



def main():
    print(__file__ + " start!!")
    # test_optimize_path()
    trajs = GenerateTrajectories()

    # all trajs list of list of trajectories, which are list of tuples
    allTrajs= []
    for key in trajs:
        allTrajs.append(trajs[key])

    # Generate json file
    # For each speed, multiple trajectories.
    # Each trajectory is a list of tuples
    path = os.getcwd()
    actionPath = path + '/'
    actionName = 'actions'

    ExportJson.WriteJson(actionPath, actionName, allTrajs)

    actionsTemp = ExportJson.ReadJson(actionPath, actionName)
    actionsBack = []
    for actionHeadingList in actionsTemp:
        tempList = []
        for path in actionHeadingList:
            tempList.append(path)

        actionsBack.append(tempList)

    # compare actionsBack and allTrajs
    # for i in range(len(allTrajs)):
    #     for j in range(len(allTrajs[i])):
    #         print("Original", allTrajs[i][j])
    #         print("Actions Back", actionsBack[i][j])


if __name__ == '__main__':
    main()
