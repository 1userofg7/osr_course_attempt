import numpy as np
import openravepy as orpy
import math
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
import time


np.random.seed(4)

# Environment stuff
env = orpy.Environment() # create the environment
env.Load('osr_openrave/worlds/pick_and_place.env.xml')
env.SetViewer('qtcoin')
# orpy.RaveSetDebugLevel(orpy.DebugLevel.Debug) # set output level to debug


def create_box(T, color = [0, 0.6, 0]):
  box = orpy.RaveCreateKinBody(env, '')
  box.SetName('box')
  box.InitFromBoxes(np.array([[0,0,0,0.035,0.03,0.005]]), True)
  g = box.GetLinks()[0].GetGeometries()[0]
  g.SetAmbientColor(color)
  g.SetDiffuseColor(color)
  box.SetTransform(T)
  env.Add(box,True)
  return box


T = np.eye(4)
container_center = np.array([0.4, 0.2, 0.195])
# Destination
T[:3, 3] = container_center + np.array([0, -0.5, 0])
destination0 = create_box(T, color = [0, 0, 0.6])
T[:3, 3] = container_center + np.array([0, -0.6, 0])
destination1 = create_box(T, color = [0, 0, 0.6])

# Generate random box positions
boxes = []
nbox_per_layer = 2
n_layer = 20
h = container_center[2]
for i in range(n_layer):
    nbox_current_layer = 0
    while nbox_current_layer < nbox_per_layer:
        theta = np.random.rand()*np.pi
        T[0, 0] = np.cos(theta)
        T[0, 1] = -np.sin(theta)
        T[1, 0] = np.sin(theta)
        T[1, 1] = np.cos(theta)
        T[0, 3] = container_center[0] + (np.random.rand()-0.5)*0.2
        T[1, 3] = container_center[1] + (np.random.rand()-0.5)*0.1
        T[2, 3] = h
        box = create_box(T)
        if env.CheckCollision(box):
            env.Remove(box)
        else:
            boxes.append(box)
            nbox_current_layer += 1
    h += 0.011


#Set Robot
robot = env.GetRobots()[0]
manipulator = robot.SetActiveManipulator('gripper')
robot.SetActiveDOFs(manipulator.GetArmIndices())
taskmanip = orpy.interfaces.TaskManipulation(robot)
np.set_printoptions(precision=6, suppress=True)
robot.SetDOFVelocityLimits(robot.GetDOFVelocityLimits()*0.4)
robot.SetDOFAccelerationLimits(robot.GetDOFAccelerationLimits()*0.2)
manipprob = orpy.interfaces.BaseManipulation(robot)

#For constraints manipulations
link_idx = [l.GetName() for l in robot.GetLinks()].index('robotiq_85_base_link')
link_origin = robot.GetLink('robotiq_85_base_link').GetTransform()[:3,3]

#IK model
ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=orpy.IkParameterization.Type.Transform6D)
if not ikmodel.load():
  ikmodel.autogenerate()

statsmodel = orpy.databases.linkstatistics.LinkStatisticsModel(robot)
if not statsmodel.load():
  rospy.loginfo('Generating LinkStatistics database. It will take around 1 minute...')
  statsmodel.autogenerate()
statsmodel.setRobotWeights()
statsmodel.setRobotResolutions(xyzdelta=0.01)

# Initialize figure to plot tilt angle
fig = Figure()
canvas = FigureCanvas(fig)
ax = fig.add_subplot(1,1,1)

n_box_transferred=40
h_add=0.011 #10mm for box + 1mm for spacing
dest0_i=0
dest1_i=0

displace_step=0.002    #max 10 times on each side of box

gripchange_no=0
slide_no=0

def Boxedge_move(T_g,d_v):
    transl = np.eye(4)
    transl[:3,3]=np.array([d_v, 0, 0])
    T_g=np.dot(T_g,transl)
    return T_g

def Rotz(T_g,theta=np.pi/2):
    rotmat=np.array([[np.cos(theta),-np.sin(theta),0.],[np.sin(theta),np.cos(theta),0.],[0.,0.,1.]])
    T_plch=np.dot(T_g[:3,:3],rotmat)
    T_g[:3,:3]=T_plch
    return T_g

def Gripper_Loc(box_i,z_offset=0.005,gripside_c=False,d_v=0):
    T_g=box_i.GetTransform()
    b_c=box_i.ComputeAABB().pos()
    T_g[:3,0]*=-1   #rotate by pi along y-axis first then transform (rotz) to match box
    T_g[:3,2]*=-1   #left multiplication (order of transform from right to left)
    T_g[:3,3]=b_c
    T_g[2,3]+=z_offset
    if gripside_c==True:
        T_g=Rotz(T_g,np.pi/2)
        T_g=Boxedge_move(T_g,d_v)
    else:
        T_g=Boxedge_move(T_g,d_v)
    return T_g

def Move_Gripper(q_dest):
    # Create the planner
    planner = orpy.RaveCreatePlanner(env, 'birrt') # Using bidirectional RRT
    params = orpy.Planner.PlannerParameters()
    params.SetRobotActiveJoints(robot)
    params.SetGoalConfig(q_dest)
    params.SetPostProcessing('ParabolicSmoother', '<_nmaxiterations>40</_nmaxiterations>')
    success = planner.InitPlan(robot, params)
    if not success:
        return None
    # Plan a trajectory
    tj = orpy.RaveCreateTrajectory(env, '')
    status = planner.PlanPath(tj)
    if status != orpy.PlannerStatus.HasSolution:
        return None
    # Execute the trajectory
    return tj

def Act_Gripper(box_i,grip=True):
    if grip==True:
        robot.WaitForController(0)
        robot.Grab(box_i)
    else:
        robot.WaitForController(0)
        robot.Release(box_i)

def transxyz(xyzt_value=0.001,xyz_axis=0.,q_curr=None):
    J = np.zeros((6,6))
    if xyz_axis==0:
        twist = np.array([xyzt_value, 0, 0, 0, 0, 0])
    elif xyz_axis==1:
        twist = np.array([0, xyzt_value, 0, 0, 0, 0])
    elif xyz_axis==2:
        twist = np.array([0, 0, xyzt_value, 0, 0, 0])
    J[:3,:] = robot.ComputeJacobianTranslation(link_idx, link_origin)[:,:6]
    J[3:,:] = robot.ComputeJacobianAxisAngle(link_idx)[:,:6]
    qdot = np.linalg.solve(J, twist)
    q_curr[:6] += qdot
    return q_curr

def rotxyz(xyzt_value=0.001,xyz_axis=0.,q_curr=None):
    J = np.zeros((6,6))
    if xyz_axis==0:
        twist = np.array([0, 0, 0, xyzt_value, 0, 0])
    elif xyz_axis==1:
        twist = np.array([0, 0, 0, 0, xyzt_value, 0])
    elif xyz_axis==2:
        twist = np.array([0, 0, 0, 0, 0, xyzt_value])
    J[:3,:] = robot.ComputeJacobianTranslation(link_idx, link_origin)[:,:6]
    J[3:,:] = robot.ComputeJacobianAxisAngle(link_idx)[:,:6]
    qdot = np.linalg.solve(J, twist)
    q_curr[:6] += qdot
    return q_curr

def new_loc_transl(xyz_axis=0,displace_xyz=0,q_curr=None):
    move_step=0.002
    if displace_xyz<0:
        move_step=-0.002
    curr_disp=0.
    abs_displace_xyz=abs(displace_xyz)
    count_move=1
    while curr_disp<abs_displace_xyz:
        q_curr=transxyz(move_step,xyz_axis,q_curr)
        manipprob.MoveManipulator(goal=q_curr)
        robot.WaitForController(0)
        curr_disp+=abs(move_step)
        count_move+=1
    return q_curr

def new_loc_rot(xyz_axis=0,displace_rot=0,q_curr=None):
    move_step=0.02
    if displace_rot<0:
        move_step=-0.02
    curr_disp=0.
    abs_displace_rot=abs(displace_rot)
    while curr_disp<abs_displace_rot:
        q_curr=rotxyz(move_step,xyz_axis,q_curr)
        manipprob.MoveManipulator(goal=q_curr)
        robot.WaitForController(0)
        curr_disp+=abs(move_step)
    return q_curr

def rot_neutral(box_i,z_offset=0.005,gripside_c=False,d_v=0):
    T_g=np.eye(4)
    b_c=box_i.ComputeAABB().pos()
    T_g[:3,0]*=-1   #rotate by pi along y-axis first then transform (rotz) to match box
    T_g[:3,2]*=-1   #left multiplication (order of transform from right to left)
    T_g[:3,3]=b_c
    T_g[2,3]+=z_offset
    if gripside_c==True:
        T_g=Rotz(T_g,np.pi/2)
        T_g=Boxedge_move(T_g,d_v)
    else:
        T_g=Boxedge_move(T_g,d_v)
    return T_g

i_jacob=raw_input("Enter number of jacobian axis movements (0-5): ")
i_jacob=int(i_jacob)
#print i_jacob
i_boxmove=raw_input("Enter number of boxes to be moved (1-40): ")
i_boxmove=int(i_boxmove)
#print i_boxmove

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
raw_input("Press Enter to start pick & place")
#~~~~~~~~~~~~~~~START PROGRAM~~~~~~~~~~~~~~~~
start_time = time.time()

#Sort boxes according to decreasing z-axis value
boxes.sort(key=lambda box:box.GetTransform()[2, 3], reverse=True)

for i in range(i_boxmove):
    #raw_input("Press Enter for box %d" %i)
    displace_value=0.0
    grip_changed=False
    if i<39:
        box_centroid=boxes[i].ComputeAABB().pos()
        boxn_centroid=boxes[i+1].ComputeAABB().pos()
        if box_centroid[2]==boxn_centroid[2]:   #same height
            if box_centroid[0] > boxn_centroid[0]:
                box_plch=boxes[i]
                boxes[i]=boxes[i+1]
                boxes[i+1]=box_plch
    
    # Find soln for Tgrasp
    Tgrasp=Gripper_Loc(boxes[i])
    Tgrasp_new=Gripper_Loc(boxes[i])
    qgrasp=manipulator.FindIKSolution(Tgrasp_new, orpy.IkFilterOptions.CheckEnvCollisions)
    #move gripper along axis parallel to gripping pads, with height (z) constant
    if qgrasp is None:
        print("Unable to find soln: sliding gripper//side")
        for j in range(1,11):
            Tgrasp_new=Boxedge_move(Tgrasp,displace_step*j)
            qgrasp=manipulator.FindIKSolution(Tgrasp_new, orpy.IkFilterOptions.CheckEnvCollisions)
            if qgrasp is not None:
                displace_value=displace_step*j
                print("Found Soln: slide box %d at %.2f mm" % (i,displace_step*j*1000))
                slide_no+=1
                break
            else:
                Tgrasp_new=Boxedge_move(Tgrasp,-displace_step*j)
                qgrasp=manipulator.FindIKSolution(Tgrasp_new, orpy.IkFilterOptions.CheckEnvCollisions)
                if qgrasp is not None:
                    displace_value=-displace_step*j
                    print("Found Soln: slide box %d at %.2f mm" % (i,-displace_step*j*1000))
                    slide_no+=1
                    break
                else:
                    continue
    else:
        print("Found Soln: no iterations needed")

    #change gripping side on box if qgrasp still None
    if qgrasp is None:
        print("Unable to find soln: changing grip side")
        grip_changed=True
        Tgrasp=Rotz(Tgrasp)
        Tgrasp_new=Rotz(Tgrasp)
        qgrasp=manipulator.FindIKSolution(Tgrasp_new, orpy.IkFilterOptions.CheckEnvCollisions)
        if qgrasp is None:
            print("Unable to find soln: sliding gripper//side")
            for j in range(1,11):
                Tgrasp_new=Boxedge_move(Tgrasp,displace_step*j)
                qgrasp=manipulator.FindIKSolution(Tgrasp_new, orpy.IkFilterOptions.CheckEnvCollisions)
                if qgrasp is not None:
                    displace_value=displace_step*j
                    print("Found Soln: slide box %d at %.2f mm" % (i,displace_step*j*1000))
                    slide_no+=1
                    break
                else:
                    Tgrasp_new=Boxedge_move(Tgrasp,-displace_step*j)
                    qgrasp=manipulator.FindIKSolution(Tgrasp_new, orpy.IkFilterOptions.CheckEnvCollisions)
                    if qgrasp is not None:
                        displace_value=-displace_step*j
                        print("Found Soln: slide box %d at %.2f mm" % (i,-displace_step*j*1000))
                        slide_no+=1
                        break
                    else:
                        continue
        gripchange_no+=1
    if qgrasp is None:
        print("No solutions found for box %d" %i)
        env.Remove(boxes[i])
        n_box_transferred-=1
        continue

    #print("Tgrasp = ", Tgrasp_new)
    #print("Solution = ", qgrasp)

    # Find soln for Trelease
    if dest1_i>dest0_i:
        dest_rel=destination0
        dest_i=dest0_i
        dest_chosen=0
    else:
        dest_rel=destination1
        dest_i=dest1_i
        dest_chosen=1

    print("Finding soln for box %d at destination %d" %(i,dest_chosen))

    Trel=Gripper_Loc(dest_rel,0.016+h_add*dest_i,grip_changed,displace_value)
    Trel_new=Gripper_Loc(dest_rel,0.016+h_add*dest_i,grip_changed,displace_value)
    qrel=manipulator.FindIKSolution(Trel_new, orpy.IkFilterOptions.CheckEnvCollisions)
    if qrel is None:
        Trel_new=Rotz(Trel,np.pi)
        qrel=manipulator.FindIKSolution(Trel_new, orpy.IkFilterOptions.CheckEnvCollisions)
        if qrel is None:
            Trel_new=Rotz(Trel,-np.pi)
            qrel=manipulator.FindIKSolution(Trel_new, orpy.IkFilterOptions.CheckEnvCollisions)
    if qrel is None:
        print("Unable to find soln: changing destination")
        if dest_chosen==1:
            dest_rel=destination0
            dest_i=dest0_i
            dest_chosen=0
        else:
            dest_rel=destination1
            dest_i=dest1_i
            dest_chosen=1
        Trel=Gripper_Loc(dest_rel,0.016+h_add*dest_i)
        Trel_new=Gripper_Loc(dest_rel,0.016+h_add*dest_i)
        qrel=manipulator.FindIKSolution(Trel_new, orpy.IkFilterOptions.CheckEnvCollisions)
        if qrel is None:
            Trel_new=Rotz(Trel,np.pi)
            qrel=manipulator.FindIKSolution(Trel_new, orpy.IkFilterOptions.CheckEnvCollisions)
            if qrel is None:
                Trel_new=Rotz(Trel,-np.pi)
                qrel=manipulator.FindIKSolution(Trel_new, orpy.IkFilterOptions.CheckEnvCollisions)
    if qrel is None:
        print("No release Soln found for box %d" %i)
        n_box_transferred-=1
        env.Remove(boxes[i])
        continue
    if dest_chosen==0:
        dest0_i+=1
    else:
        dest1_i+=1
    
    #print("Trel = ", Trel_new)
    #print("Solution Release = ", qrel)

    traj_grasp=Move_Gripper(qgrasp)
    traj_rel=Move_Gripper(qrel)
    if traj_grasp==None:
        print("Failed grasp traj for box %d" %i)
        n_box_transferred-=1
        env.Remove(boxes[i])
        continue
    if traj_rel==None:
        print("Failed release traj for box %d" %i)
        n_box_transferred-=1
        env.Remove(boxes[i])
        continue
    #raw_input("Press Enter to move (grasp)")
    manipprob.MoveManipulator(goal=qgrasp, outputtrajobj=True)
    robot.WaitForController(0)
    #raw_input("Press Enter to grip")
    Act_Gripper(boxes[i])
    robot.WaitForController(0)

    # Attempt 1: for z-axis angle = 0 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #move x
    #raw_input("zero tilt attempt press Enter")
    if i_jacob>0:
        container_center = np.array([0.4, 0.2, 0.195])
        container_size = np.array([0.2, 0.1, 0.12])
        q_move_x=qgrasp
        T_curr=Tgrasp_new[0,3]
        T_move_x=Tgrasp_new
        T_move_x[0,3]=container_center[0]-container_size[0]/2   #new location: 1/4x of container
        displace_x=T_move_x[0,3]-T_curr
        q_move_x=new_loc_transl(0,displace_x,q_move_x)
    if i_jacob>1:
        #move z
        q_move_z=q_move_x
        T_curr=boxes[i].ComputeAABB().pos()[2]
        T_move_z=T_move_x
        T_move_z[2,3]=container_center[2]+container_size[2]*2+0.012   #new location: slightly above container top (z)
        displace_z=T_move_z[2,3]-T_curr
        q_move_z=new_loc_transl(2,displace_z,q_move_z)
    if i_jacob>2:
        #move y
        q_move_y=q_move_z
        T_curr=boxes[i].ComputeAABB().pos()[1]
        T_move_y=T_move_z
        T_move_y[1,3]=container_center[1]-container_size[1]*2-0.01   #new location: outside container (y_new<y_old)
        displace_y=T_move_y[1,3]-T_curr
        q_move_y=new_loc_transl(1,displace_y,q_move_y)
    if i_jacob>3:
        #move z (negative)
        if dest_chosen==0:
            base_level=destination0.GetTransform()[2,3]
        else:
            base_level=destination1.GetTransform()[2,3]
        q_move_z=q_move_y
        T_curr=boxes[i].ComputeAABB().pos()[2]
        T_move_z=T_move_y
        T_move_z[2,3]=base_level+0.016+h_add*(dest_i)   #new location: outside container at placement level
        displace_z=T_move_z[2,3]-T_curr
        q_move_z=new_loc_transl(2,displace_z,q_move_z)

        #rotate neutral
        #raw_input("rot")
        #q_rot_neut=q_move_z
        #T_rot_neut=rot_neutral(boxes[i],gripside_c=grip_changed,d_v=displace_value)
        #print T_move_z
        #print T_rot_neut
        #T_rot_curr=boxes[i].GetTransform()
        #displace_rz=-math.atan2(T_rot_curr[1,0],T_rot_curr[0,0])
        #print displace_rz
        #raw_input("rot2")
        #q_rot_neut=new_loc_rot(2,displace_rz,q_rot_neut)
    if i_jacob>4:
        #move y alight destination
        q_move_y=q_move_z
        T_curr=boxes[i].ComputeAABB().pos()[1]
        T_move_y=T_move_z
        if dest_chosen==0:
            T_move_y[1,3]=destination0.ComputeAABB().pos()[1]  #new location: x of destinations
        else:
            T_move_y[1,3]=destination1.ComputeAABB().pos()[1]
        displace_y=T_move_y[1,3]-T_curr
        q_move_y=new_loc_transl(1,displace_y,q_move_y)

   
    #raw_input("zero tilt attempt press Exit")
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    #raw_input("Press Enter to move (release)")
    traj=manipprob.MoveManipulator(goal=qrel, outputtrajobj=True)
    robot.WaitForController(0)
    #raw_input("Press Enter to release")
    Act_Gripper(boxes[i],False)

    times=np.arange(0, traj.GetDuration(), 0.01)
    qvect = np.zeros((len(times), robot.GetActiveDOF()))
    spec = traj.GetConfigurationSpecification()
    for j_time in range(len(times)):
        trajdata = traj.Sample(times[j_time])
        qvect[j_time,:] = spec.ExtractJointValues(trajdata, robot, manipulator.GetArmIndices(), 0)
    theta_z = np.zeros(len(times))

    #Understand (Taken from quandao github)
    overall_z=np.array([0,0,1])
    with robot:
        for j_time in range(len(times)):
            robot.SetActiveDOFValues(qvect[j_time, :])
            local_z=manipulator.GetEndEffectorTransform()[:3,2]
            mag_local=math.sqrt(local_z[0]**2+local_z[1]**2+local_z[2]**2)
            #print manipulator.GetEndEffectorTransform()[:3,3]
            #print local_z
            #print mag_local
            theta_z[j_time]=math.acos(np.dot(overall_z,local_z)/abs(1*mag_local))
            #print 180-theta_z[j_time]*180/np.pi

    ax.plot(times,180-theta_z*180/np.pi, label='box %d'%i)

print("Time taken = ", time.time() - start_time)
print("Transferred %d boxes" %n_box_transferred)
print("No. of grip changed = %d" %gripchange_no)
print("No. of box edge slide = %d" %slide_no)

# Plot tilt angles
ax.grid(True)
ax.legend()
ax.set_xlabel('Time /s')
ax.set_ylabel('Theta_z /deg')
canvas.print_figure('Tilt Angle against Time.png')

raw_input("Press Enter to finish")
