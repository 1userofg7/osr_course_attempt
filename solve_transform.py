import numpy as np
import math

def read_takefile(str_fn):
  with open(str_fn,'r') as f:
    fltxt=f.read()
  
  line12=fltxt.split("\n")
  line12[0]=line12[0].replace(']','').replace('[','')
  line12[0]=line12[0].replace(')','').replace('(','')
  line12[0]=line12[0].split(",")
  center2d=[0,0]
  center2d[0]=float(line12[0][0])
  center2d[1]=float(line12[0][1])
  line12[1]=line12[1].replace(']','').replace('[','')
  line12[1]=line12[1].split(",")
  top2d=[0,0]
  left2d=[0,0]
  bottom2d=[0,0]
  top2d[0]=float(line12[1][0])
  top2d[1]=float(line12[1][1])
  left2d[0]=float(line12[1][2])
  left2d[1]=float(line12[1][3])
  bottom2d[0]=float(line12[1][4])
  bottom2d[1]=float(line12[1][5])
  return center2d,top2d,left2d,bottom2d

def cameras_projmat():
  import yaml
  left_calib_data = yaml.load( open("left_camera_info.yaml", "r"))
  left_cam_matrix = left_calib_data["P"] 
  right_calib_data = yaml.load( open("right_camera_info.yaml", "r"))
  right_cam_matrix = right_calib_data["P"] 
  # Compute projection matrix Q
  Tx = right_cam_matrix[3]
  fx = right_cam_matrix[0]
  B = (-Tx / fx)
  lCx = left_cam_matrix[2]
  lCy = left_cam_matrix[6]
  rCx = right_cam_matrix[2]
  rCy = right_cam_matrix[6]
  Q = np.zeros((4,4))
  Q[0,0] = 1
  Q[1,1] = 1
  Q[3,2] = 1./B
  Q[0,3] = -lCx
  Q[1,3] = -lCy
  Q[2,3] = fx
  Q[3,3] = (rCx-lCx)/B
  return Q

def find_3dcoord(L2d,R2d,Qproj):
  disparity = L2d[0] - R2d[0]
  XYZ = np.dot(Qproj, np.array([L2d[0], L2d[1], disparity, 1]))
  XYZ /= XYZ[-1]
  return XYZ[:3]

def find_conline(linept1,linept2):	#find connecting line
  line_reslt=np.zeros(3)
  for dimsn in range(3):
    line_reslt[dimsn]=linept1[dimsn]-linept2[dimsn]
  return line_reslt

def mag_line(lineM):
  return math.sqrt(lineM[0]**2+lineM[1]**2+lineM[2]**2)

def angle_btwn(lineA,lineB):
  return np.arccos(np.dot(lineA,lineB)/(mag_line(lineA)*mag_line(lineB)))

def compute_R(r_axis,r_angle):
  R=np.zeros((3,3))
  r_axis_mag=mag_line(r_axis)
  r_x=r_axis[0]/r_axis_mag
  r_y=r_axis[1]/r_axis_mag
  r_z=r_axis[2]/r_axis_mag
  cos_ra=math.cos(r_angle)
  sin_ra=math.sin(r_angle)
  R[0,0]=cos_ra+(r_x**2)*(1-cos_ra)
  R[0,1]=r_x*r_y*(1-cos_ra)-r_z*sin_ra
  R[0,2]=r_x*r_z*(1-cos_ra)+r_y*sin_ra
  R[1,0]=r_y*r_x*(1-cos_ra)+r_z*sin_ra
  R[1,1]=cos_ra+(r_y**2)*(1-cos_ra)
  R[1,2]=r_y*r_z*(1-cos_ra)-r_x*sin_ra
  R[2,0]=r_z*r_x*(1-cos_ra)-r_y*sin_ra
  R[2,1]=r_z*r_y*(1-cos_ra)+r_x*sin_ra
  R[2,2]=cos_ra+(r_z**2)*(1-cos_ra)
  return R

def compute_perpintrs(point_c,vector_v,point_p):	#perpendicular intersect of point to line
  num_eq=point_p[0]*vector_v[0]+point_p[1]*vector_v[1]+point_p[2]*vector_v[2]
  num_eq=num_eq-vector_v[0]*point_c[0]-vector_v[1]*point_c[1]-vector_v[2]*point_c[2]
  denom_eq=vector_v[0]**2+vector_v[1]**2+vector_v[2]**2
  t_line=num_eq/denom_eq
  intrspt=np.zeros(3)
  for i_axis in range(3):
    intrspt[i_axis]=point_c[i_axis]+t_line*vector_v[i_axis]
  return intrspt

def conv_unitv(lineU):
  lineU_mag=mag_line(lineU)
  for dimsn in range(3):
    lineU[dimsn]=lineU[dimsn]/lineU_mag
  return lineU

def gplane_pose(lv_dat,rv_dat):
  center2dl,top2dl,left2dl,bottom2dl=read_takefile(lv_dat)
  center2dr,top2dr,left2dr,bottom2dr=read_takefile(rv_dat)
  Q=cameras_projmat()
  center3d = find_3dcoord(center2dl,center2dr,Q)
  top3d = find_3dcoord(top2dl,top2dr,Q)
  left3d = find_3dcoord(left2dl,left2dr,Q)
  bottom3d = find_3dcoord(bottom2dl,bottom2dr,Q)
  #print top3d,center3d,bottom3d
  line_parll1=np.zeros(3)
  line_parll1=find_conline(top3d,center3d)
  line_parll2=np.zeros(3)
  line_parll2=find_conline(left3d,center3d)
  line_norm=np.cross(line_parll1,line_parll2)
  ooscreen=np.array([0,0,1])
  rot_axis=np.cross(line_norm,ooscreen)
  rot_angle=angle_btwn(ooscreen,line_norm)
  #print rot_angle
  R=compute_R(rot_axis,rot_angle)
  R1=compute_R(rot_axis,-rot_angle)
  #p_intersect=compute_perpintrs(center3d,rot_axis,left3d)
  #p_vector=find_conline(left3d,p_intersect)
  #p_mag=mag_line(p_vector)
  #new_p_vector=np.matmul(R,p_vector)
  #print mag_line(new_p_vector),p_mag
  #new_p_vector=conv_unitv(new_p_vector)
  #new_p_vector=p_mag*new_p_vector
  #new_left3d=find_conline(p_intersect,-new_p_vector)	#negative used to add the vector to pt
  #theta_z=-math.atan2(new_left3d[1]-center3d[1],center3d[0]-new_left3d[0])	#reversed as left is opp of local x_axis
  #add
  new_p_vector=np.matmul(R,line_parll2)
  #print mag_line(new_p_vector),p_mag
  new_p_vector=conv_unitv(new_p_vector)
  #calc rot (earlier)
  rot_axis=np.cross(new_p_vector,np.array([-1,0,0]))
  rot_angle=angle_btwn(np.array([-1,0,0]),new_p_vector)
  R2=compute_R(rot_axis,-rot_angle)
  #addend
  #R2=np.array([[math.cos(theta_z),-math.sin(theta_z),0.],\
  #[math.sin(theta_z),math.cos(theta_z),0.],[0.,0.,1.]])
  #print theta_z
  #print rot_angle
  R_overall=np.matmul(R1,R2)
  plane_pose=np.eye(4)
  plane_pose[:3,:3]=R_overall
  plane_pose[:3,3]=center3d
  #print new_p_vector
  #print np.dot(new_p_vector,rot_axis)
  #print np.matmul(R,line_norm)
  #print new_left3d, center3d
  #print plane_pose
  return plane_pose

def transform_pose(lv_dat,rv_dat,l_ogdat,r_ogdat):
  #dat for start plane
  center2dl,top2dl,left2dl,bottom2dl=read_takefile(lv_dat)
  center2dr,top2dr,left2dr,bottom2dr=read_takefile(rv_dat)
  Q=cameras_projmat()
  s_center3d = find_3dcoord(center2dl,center2dr,Q)
  s_top3d = find_3dcoord(top2dl,top2dr,Q)
  s_left3d = find_3dcoord(left2dl,left2dr,Q)
  s_bottom3d = find_3dcoord(bottom2dl,bottom2dr,Q)
  #print top3d,center3d,bottom3d
  s_line_parll1=np.zeros(3)
  s_line_parll1=find_conline(s_top3d,s_center3d)
  s_line_parll2=np.zeros(3)
  s_line_parll2=find_conline(s_left3d,s_center3d)
  s_line_norm=np.cross(s_line_parll1,s_line_parll2)
  #dat for destination plane
  center2dl,top2dl,left2dl,bottom2dl=read_takefile(l_ogdat)
  center2dr,top2dr,left2dr,bottom2dr=read_takefile(r_ogdat)
  d_center3d = find_3dcoord(center2dl,center2dr,Q)
  d_top3d = find_3dcoord(top2dl,top2dr,Q)
  d_left3d = find_3dcoord(left2dl,left2dr,Q)
  d_bottom3d = find_3dcoord(bottom2dl,bottom2dr,Q)
  #print top3d,center3d,bottom3d
  d_line_parll1=np.zeros(3)
  d_line_parll1=find_conline(d_top3d,d_center3d)
  d_line_parll2=np.zeros(3)
  d_line_parll2=find_conline(d_left3d,d_center3d)
  d_line_norm=np.cross(d_line_parll1,d_line_parll2)
  #calc translation
  transl_0=np.eye(4)
  transl_0[:3,3]=find_conline(s_center3d,d_center3d)
  #calc rot (later)
  rot_axis=np.cross(s_line_norm,d_line_norm)
  rot_angle=angle_btwn(d_line_norm,s_line_norm)
  R=compute_R(rot_axis,rot_angle)
  R1=np.eye(4)
  R1[:3,:3]=compute_R(rot_axis,-rot_angle)
  new_p_vector=np.matmul(R,s_line_parll2)
  #print mag_line(new_p_vector),p_mag
  new_p_vector=conv_unitv(new_p_vector)
  #calc rot (earlier)
  rot_axis=np.cross(new_p_vector,d_line_parll2)
  rot_angle=angle_btwn(d_line_parll2,new_p_vector)
  R2=np.eye(4)
  R2[:3,:3]=compute_R(rot_axis,-rot_angle)
  T_overall=np.matmul(R1,R2)
  T_overall[:3,3]=transl_0[:3,3]
  return T_overall

def transform_mat(mat_transf, mat_main):
  mat_reslt=np.eye(4)
  mat_reslt[:3,3]=np.add(mat_main[:3,3],mat_transf[:3,3])
  mat_reslt[:3,:3]=np.matmul(mat_transf[:3,:3],mat_main[:3,:3])
  return mat_reslt

# Main Programme starts here~~~~~~~~~~~
#find pose of take1
lv_ogdat='take1l_test.txt'
rv_ogdat='take1r_test.txt'

p_p1=gplane_pose(lv_ogdat,rv_ogdat)
print("Pose 1:")
print p_p1
print ("\n")

#find T2 to T4
leftview_dat='take2l_test.txt'
rightview_dat='take2r_test.txt'
T2=transform_pose(leftview_dat,rightview_dat,lv_ogdat,rv_ogdat)
print("Transform 2:")
print T2
p_p2=transform_mat(T2, p_p1)
print("Pose 2:")
print p_p2
print ("\n")

leftview_dat='take3l_test.txt'
rightview_dat='take3r_test.txt'
T3=transform_pose(leftview_dat,rightview_dat,lv_ogdat,rv_ogdat)
print("Transform 3:")
print T3
p_p3=transform_mat(T3, p_p1)
print("Pose 3:")
print p_p3
print ("\n")

leftview_dat='take4l_test.txt'
rightview_dat='take4r_test.txt'
T4=transform_pose(leftview_dat,rightview_dat,lv_ogdat,rv_ogdat)
print("Transform 4:")
print T4
p_p4=transform_mat(T4, p_p1)
print("Pose 4:")
print p_p4
print ("\n")

