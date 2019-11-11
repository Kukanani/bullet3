import inspect
import os

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print ("current_dir=" + currentdir)
parentdir = os.path.join(currentdir,"../gym")

os.sys.path.insert(0,parentdir)

import pybullet as p
import pybullet_data
import numpy as np

from copy import deepcopy

gui = False

inactive_wheels = [3, 5, 7]
wheels = [2]
steering = [4,6]

def test_car(weight_position):
    if gui:
        cid = p.connect(p.SHARED_MEMORY)
        if (cid < 0):
            p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)
        print("connected")

    p.setRealTimeSimulation(False)
    p.resetSimulation()
    p.setGravity(0, 0, -10)
    p.loadSDF(os.path.join(pybullet_data.getDataPath(), "stadium.sdf"))

    pos_store = np.empty([1000, 3])
    ori_store = np.empty([1000, 4])

    car = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"racecar/racecar.urdf"), [0, 0, 0])

    cube_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])
    mass = 0.1
    cube = p.createMultiBody(mass, cube_shape, cube_shape, basePosition=weight_position)
    p.createConstraint(cube, -1, car, -1, p.JOINT_FIXED, [0, 0, 0], -weight_position, [0, 0, 0])

    for wheel in inactive_wheels:
        p.setJointMotorControl2(car,wheel,p.VELOCITY_CONTROL,targetVelocity=0,force=0)

    # targetVelocitySlider = p.addUserDebugParameter("wheelVelocity",-10,10,0)
    # maxForceSlider = p.addUserDebugParameter("maxForce",0,10,10)
    # steeringSlider = p.addUserDebugParameter("steering",-0.5,0.5,0)

    for i in range(1000):
        # maxForce = p.readUserDebugParameter(maxForceSlider)
        # targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
        # steeringAngle = p.readUserDebugParameter(steeringSlider)

        maxForce = 10
        targetVelocity = 10
        steeringAngle = 0.4

        for wheel in wheels:
            p.setJointMotorControl2(car,wheel,p.VELOCITY_CONTROL,targetVelocity=targetVelocity,force=maxForce)

        for steer in steering:
            p.setJointMotorControl2(car,steer,p.POSITION_CONTROL,targetPosition=steeringAngle)

        p.stepSimulation()
        # time.sleep(1/240)

        pos, ori = p.getBasePositionAndOrientation(car)
        pos_store[i, :] = deepcopy(pos)
        ori_store[i, :] = deepcopy(ori)

    print("disconnecting")
    p.disconnect()
    return pos_store, ori_store

def main():
    num_divisions = 5

    pos_set = np.empty([num_divisions, 1000, 3])
    ori_set = np.empty([num_divisions, 1000, 4])

    y_range = 0.2
    for i in range(num_divisions):
        y_offset = -(y_range/2) + y_range/(num_divisions-1)*i
        print("car y-offset is: " + str(y_offset))
        pos_set[i], ori_set[i] = test_car(np.array([0.2, y_offset, 0.2]))


if __name__ == "__main__":
    main()