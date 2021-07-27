import openravepy as orpy
env = orpy.Environment()
env.Load('osr_openrave/robots/denso_robotiq_85_gripper.robot.xml')
env.SetDefaultViewer()
robot = env.GetRobot('denso_robotiq_85_gripper')
manipulator = robot.SetActiveManipulator('gripper')
robot.SetActiveDOFs(manipulator.GetArmIndices())

robot.SetActiveDOFValues([0.1, 0.7, 1.5, -0.5, -0.8, -1.2])

robot.GetLinks()

link_idx = [l.GetName() for l in robot.GetLinks()].index('robotiq_85_base_link')
link_origin = robot.GetLink('robotiq_85_base_link').GetTransform()[:3,3]
# Improve the visualization settings
import numpy as np
np.set_printoptions(precision=6, suppress=True)
# Print the result
print robot.ComputeJacobianTranslation(link_idx, link_origin)

print robot.ComputeJacobianAxisAngle(link_idx)
