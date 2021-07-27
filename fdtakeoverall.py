#cd ~/catkin_ws/src/osr_course_pkgs/osr_examples/vision_assignment_data

import cv2
import numpy as np
import math
from matplotlib import pyplot as plt

def group_lines(seg_lines,interc_tol):
  line_grp=[[]]	#group lines according to same grad and y_interc
  avg_gy=[[]]	#avg grad and y_interc
  x_intstore=[[]]
  for i in range(len(seg_lines)):
    if (seg_lines[i,0,0]-seg_lines[i,0,2])==0:
      grad_seg=np.pi/2
    else:
      grad_seg=math.atan(float(seg_lines[i,0,1]-seg_lines[i,0,3])/(seg_lines[i,0,0]-seg_lines[i,0,2]))
    y_interc=seg_lines[i,0,1]-math.tan(grad_seg)*seg_lines[i,0,0]
    if math.tan(grad_seg)==0:
      x_interc=999999
    else:
      x_interc=-y_interc/math.tan(grad_seg)
    if i==0:
      line_grp[0].append([seg_lines[i,0,0],seg_lines[i,0,1],seg_lines[i,0,2],seg_lines[i,0,3]])
      avg_gy[0].append(grad_seg)
      avg_gy[0].append(y_interc)
      x_intstore[0].append(x_interc)
    else:
      found_gymatch=0
      for j_avg in range(len(avg_gy)):
        if abs(avg_gy[j_avg][0]-grad_seg)<(5*np.pi/180): 
          if abs(avg_gy[j_avg][0])<(np.pi/4):
            if abs(avg_gy[j_avg][1]-y_interc)<interc_tol:	#50 - take1,2,3, 100 - take4
              avg_gy[j_avg][0]=(avg_gy[j_avg][0]*len(line_grp[j_avg])+grad_seg)/(len(line_grp[j_avg])+1)	#recalculate avg
              avg_gy[j_avg][1]=(avg_gy[j_avg][1]*len(line_grp[j_avg])+y_interc)/(len(line_grp[j_avg])+1)
              x_intstore[j_avg][0]=(x_intstore[j_avg][0]*len(line_grp[j_avg])+x_interc)/(len(line_grp[j_avg])+1)
              line_grp[j_avg].append([seg_lines[i,0,0],seg_lines[i,0,1],seg_lines[i,0,2],seg_lines[i,0,3]])
              found_gymatch=1
              break
          else:
            if abs(x_intstore[j_avg][0]-x_interc)<interc_tol:
              avg_gy[j_avg][0]=(avg_gy[j_avg][0]*len(line_grp[j_avg])+grad_seg)/(len(line_grp[j_avg])+1)	#recalculate avg
              avg_gy[j_avg][1]=(avg_gy[j_avg][1]*len(line_grp[j_avg])+y_interc)/(len(line_grp[j_avg])+1)
              x_intstore[j_avg][0]=(x_intstore[j_avg][0]*len(line_grp[j_avg])+x_interc)/(len(line_grp[j_avg])+1)
              line_grp[j_avg].append([seg_lines[i,0,0],seg_lines[i,0,1],seg_lines[i,0,2],seg_lines[i,0,3]])
              found_gymatch=1
              break
            
      if found_gymatch==0:
        line_grp.append([[seg_lines[i,0,0],seg_lines[i,0,1],seg_lines[i,0,2],seg_lines[i,0,3]]])
        avg_gy.append([grad_seg,y_interc])
        x_intstore.append([x_interc])
  avg_gy=[x for _,x in sorted(zip(line_grp,avg_gy),key=lambda pair:len(pair[0]),reverse=True)]
  line_grp.sort(key=len,reverse=True)
  return avg_gy,line_grp
  
def find_maxmin(val_list1,val_list2):
  maxval1=max(val_list1)
  maxval2=max(val_list2)
  if maxval2>maxval1:
    maxval=maxval2
  else:
    maxval=maxval1
  minval1=min(val_list1)
  minval2=min(val_list2)
  if minval2<minval1:
    minval=minval2
  else:
    minval=minval1
  return maxval,minval

def recalc_gy(avg_gy):
  for i_re in range(len(avg_gy)):
    avg_gy[i_re][0]=math.atan(float(avg_gy[i_re][3]-avg_gy[i_re][5])/(avg_gy[i_re][2]-avg_gy[i_re][4]))
    avg_gy[i_re][1]=avg_gy[i_re][3]-math.tan(avg_gy[i_re][0])*avg_gy[i_re][2]
  return avg_gy

def list_obtain(threeD_list,id0,id2):
  new_list=[id1[id2] for id1 in threeD_list[id0]]
  return new_list
  
def find_sigedge(avg_gy,e_cdrd,extreme_axis,maxmin_bol):	#extreme_axis = 0 for x / 1 for y	#maxmin_bol = 0 for min / 1 for max
  i_ec=0
  for i_extreme in range(len(avg_gy)):
    if i_extreme not in e_cdrd:
      avg_ea=(avg_gy[i_extreme][extreme_axis+2]+avg_gy[i_extreme][extreme_axis+4])/2
      if maxmin_bol==0:
        if i_ec==0:
          min_ea=avg_ea
          id_mea=i_extreme
          i_ec+=1
        elif avg_ea<min_ea:
          min_ea=avg_ea
          id_mea=i_extreme
        else:
          continue
      elif maxmin_bol==1:
        if i_ec==0:
          max_ea=avg_ea
          id_mea=i_extreme
          i_ec+=1
        elif avg_ea>max_ea:
          max_ea=avg_ea
          id_mea=i_extreme
        else:
          continue
      else:
        raw_input("Invalid input for maxmin_bool in find_sigedge")
  return id_mea

def find_parallel(avg_gy,e_cdrd,parll1_tol):
  parll_counter=0
  parll_pairs=[[]]
  for i_parll in range(len(avg_gy)):
    if i_parll not in e_cdrd:
      anglle1=avg_gy[i_parll][0]
      for j_parll in range(len(avg_gy)):
        if (j_parll not in e_cdrd) and (i_parll!=j_parll):
          anglle2=avg_gy[j_parll][0]
          diffanglle=abs(anglle1-anglle2)
          #print i_parll,j_parll,diffanglle
          if diffanglle>(np.pi/2):
            diffanglle=np.pi-diffanglle
          #print diffanglle
          if diffanglle<(parll1_tol*np.pi/180):	#take1:10, take2,3,4:20
            if parll_counter==0:
              parll_pairs[0].append(i_parll)
              parll_pairs[0].append(j_parll)
              parll_counter+=1
            else:
              parll_pairs.append([i_parll,j_parll])
              parll_counter+=1
  #print parll_pairs
  if parll_counter==0:
    raw_input("no parll lines found - adjust tolerance")
    return None, None
  if parll_counter==1:
    return parll_pairs[0][0], parll_pairs[0][1]
  for i_pair in range(parll_counter-1):
    if parll_pairs[i_pair][0]==parll_pairs[i_pair+1][0]:
      if line_length(avg_gy[parll_pairs[i_pair][1]])>line_length(avg_gy[parll_pairs[i_pair+1][1]]):
        return parll_pairs[i_pair][0], parll_pairs[i_pair][1]
      else:
        return parll_pairs[i_pair+1][0], parll_pairs[i_pair+1][1]
  return parll_pairs[0][0], parll_pairs[0][1]

def find_parallel2(avg_gy,e_cdrd,parll2_tol):
  parll_counter=0
  parll_pairs=[[]]
  for i_parll in range(len(avg_gy)):
    if i_parll not in e_cdrd:
      anglle1=avg_gy[i_parll][0]
      for j_parll in range(len(avg_gy)):
        if (j_parll not in e_cdrd) and (i_parll!=j_parll):
          anglle2=avg_gy[j_parll][0]
          diffanglle=abs(anglle1-anglle2)
          #print i_parll,j_parll,diffanglle
          if diffanglle>(np.pi/2):
            diffanglle=np.pi-diffanglle
          #print diffanglle
          if diffanglle<(parll2_tol*np.pi/180):	#take1:45, take2:45, take3:20, take4:35/45
            if parll_counter==0:
              parll_pairs[0].append(i_parll)
              parll_pairs[0].append(j_parll)
              parll_counter+=1
            else:
              parll_pairs.append([i_parll,j_parll])
              parll_counter+=1
  if parll_counter==0:
    raw_input("no parll lines found - adjust tolerance")
    return None, None
  for i_pair in range(parll_counter):
    pairdist=avg_2ldist(avg_gy[parll_pairs[i_pair][0]],avg_gy[parll_pairs[i_pair][1]])
    if i_pair==0:
      max_pairdist=pairdist
      max_pairid1=parll_pairs[i_pair][0]
      max_pairid2=parll_pairs[i_pair][1]
    else:
      if pairdist>max_pairdist:
        max_pairdist=pairdist
        max_pairid1=parll_pairs[i_pair][0]
        max_pairid2=parll_pairs[i_pair][1]
  return max_pairid1,max_pairid2

def avg_2ldist(avg_gyline1,avg_gyline2):
  midl1x=(avg_gyline1[2]+avg_gyline1[4])/2
  midl1y=(avg_gyline1[3]+avg_gyline1[5])/2
  midl2x=(avg_gyline2[2]+avg_gyline2[4])/2
  midl2y=(avg_gyline2[3]+avg_gyline2[5])/2
  return math.sqrt((midl2x-midl1x)**2+(midl2y-midl1y)**2)

def line_length(avg_gy_line):
  return math.sqrt((avg_gy_line[4]-avg_gy_line[2])**2+(avg_gy_line[3]-avg_gy_line[5])**2)

def intersect_2D(avg_gyline1,avg_gyline2):
  x_intersect=(avg_gyline2[1]-avg_gyline1[1])/(math.tan(avg_gyline1[0])-math.tan(avg_gyline2[0]))
  y_intersect=math.tan(avg_gyline1[0])*x_intersect+avg_gyline1[1]
  return x_intersect, y_intersect

def extend_planelines(avg_gyline1,avg_gyline2):
  x_intersect,y_intersect=intersect_2D(avg_gyline1,avg_gyline2)
  diff1=(avg_gyline1[2]-x_intersect)**2+(avg_gyline1[3]-y_intersect)**2
  diff2=(avg_gyline1[4]-x_intersect)**2+(avg_gyline1[5]-y_intersect)**2
  if diff1<diff2:
    avg_gyline1[2]=x_intersect
    avg_gyline1[3]=y_intersect
  else:
    avg_gyline1[4]=x_intersect
    avg_gyline1[5]=y_intersect
  diff1=(avg_gyline2[2]-x_intersect)**2+(avg_gyline2[3]-y_intersect)**2
  diff2=(avg_gyline2[4]-x_intersect)**2+(avg_gyline2[5]-y_intersect)**2
  if diff1<diff2:
    avg_gyline2[2]=x_intersect
    avg_gyline2[3]=y_intersect
  else:
    avg_gyline2[4]=x_intersect
    avg_gyline2[5]=y_intersect
  return avg_gyline1,avg_gyline2

def assign_quadrl(l_ls1,l_ls2,avg_gy,id_fl1,id_fl2,id_fs1,id_fs2,edge_sig,edge_considrd):
  global line_group
  if l_ls1>l_ls2:
    aq_cs_determined(avg_gy,id_fl1,id_fl2,id_fs1,id_fs2,edge_sig,edge_considrd)
  else:
    aq_cs_determined(avg_gy,id_fl1,id_fl2,id_fs2,id_fs1,edge_sig,edge_considrd)

def aq_cs_determined(avg_gy,id_fl1,id_fl2,id_fsc,id_fss,edge_sig,edge_considrd):	#id_fsc=curved line/id_fss=straight line 
  edge_sig[6].append(avg_gy[id_fsc])	#plane short curved
  edge_considrd.append(id_fsc)
  edge_sig[4].append(avg_gy[id_fss])	#plane short straight
  edge_considrd.append(id_fss)
  avg_x_shortc=(avg_gy[id_fsc][2]+avg_gy[id_fsc][4])/2
  avg_x_shorts=(avg_gy[id_fss][2]+avg_gy[id_fss][4])/2
  avg_y_long0=(avg_gy[id_fl1][3]+avg_gy[id_fl1][5])/2
  avg_y_long1=(avg_gy[id_fl2][3]+avg_gy[id_fl2][5])/2
  if avg_x_shortc>avg_x_shorts:	#if curved edge on the right
    if avg_y_long0<avg_y_long1:
      edge_sig[3].append(avg_gy[id_fl1])	#plane long top
      edge_considrd.append(id_fl1)
      edge_sig[5].append(avg_gy[id_fl2])	#plane long bottom
      edge_considrd.append(id_fl2)
    else:
      edge_sig[5].append(avg_gy[id_fl1])	#plane long bottom
      edge_considrd.append(id_fl1)
      edge_sig[3].append(avg_gy[id_fl2])	#plane long top
      edge_considrd.append(id_fl2)
  else:
    if avg_y_long0<avg_y_long1:
      edge_sig[5].append(avg_gy[id_fl1])	#plane long bottom
      edge_considrd.append(id_fl1)
      edge_sig[3].append(avg_gy[id_fl2])	#plane long top
      edge_considrd.append(id_fl2)
    else:
      edge_sig[3].append(avg_gy[id_fl1])	#plane long top
      edge_considrd.append(id_fl1)
      edge_sig[5].append(avg_gy[id_fl2])	#plane long bottom
      edge_considrd.append(id_fl2)

def midplane_finder(mp_edges):
  gradl1=math.atan(float(mp_edges[0][1]-mp_edges[2][1])/(mp_edges[0][0]-mp_edges[2][0]))
  y_intercl1=mp_edges[0][1]-math.tan(gradl1)*mp_edges[0][0]
  gradl2=math.atan(float(mp_edges[1][1]-mp_edges[3][1])/(mp_edges[1][0]-mp_edges[3][0]))
  y_intercl2=mp_edges[1][1]-math.tan(gradl2)*mp_edges[1][0]
  return intersect_2D([gradl1,y_intercl1],[gradl2,y_intercl2])

def find_midquad(es_quad):
  m_pts_edges=[[] for s in range(4)]
  mid_plane=0	#midplane assume to be at intersection of midpoints of edges, NOT centroid
  for i_side in range(4):
    mid_x=(es_quad[i_side][0][2]+es_quad[i_side][0][4])/2
    mid_y=(es_quad[i_side][0][3]+es_quad[i_side][0][5])/2
    m_pts_edges[i_side]=[mid_x,mid_y]
  mid_plane=[midplane_finder(m_pts_edges)]
  return m_pts_edges,mid_plane

## Main programme here~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def find_edges(image_name, coorddata, thres_val, interc_tol, parll1_tol, parll2_tol, linemin_tol):
  #set up image and probabilistic hough transform
  #thres_val = take1(L:87,R:104), take2(L:87,R:92), take3(L:70,R:75), take4(L:77,R:78)
  img = cv2.imread(image_name)
  #img = cv2.blur(image, (2,2)) 
  #img = unsharp_mask(img)
  gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
  for i in range(1024):
    for j in range(1280):
      if gray[i,j]>thres_val:
        gray[i,j] = 0
      elif gray[i,j]<thres_val+1:
        gray[i,j] = 255

#gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
  edges = cv2.Canny(gray,50,150,apertureSize = 3)
  minLineLength = 100
  maxLineGap = 10
  lines = cv2.HoughLinesP(edges,1,np.pi/1800,50,minLineLength,maxLineGap)
  #for i in range(len(lines)):
  #  cv2.line(img,(lines[i,0,0],lines[i,0,1]),(lines[i,0,2],lines[i,0,3]),(0,255,0),2)

#plt.imshow(img),plt.show()
#plt.imshow(gray),plt.show()
#cv2.imwrite('houghlines5.jpg',img)

  average_val,line_group = group_lines(lines,interc_tol)
  #print len(average_val)
  no_cl=12	#number of considered lines
  if len(average_val)<no_cl:
    print("Notice: lines found fewer than no_cl")
    no_cl=len(average_val)

  no_sl=7 #table top, left, bottom... plane top left bottom right (in order)
  for i in range(no_cl):
    x_edge_h,x_edge_l=find_maxmin(list_obtain(line_group,i,0),list_obtain(line_group,i,2))
    y_edge_h,y_edge_l=find_maxmin(list_obtain(line_group,i,1),list_obtain(line_group,i,3))
    if average_val[i][0]<0:
      average_val[i].append(x_edge_l)	#points left to right
      average_val[i].append(y_edge_h)
      average_val[i].append(x_edge_h)
      average_val[i].append(y_edge_l)
    else:
      average_val[i].append(x_edge_l)	#points left to right
      average_val[i].append(y_edge_l)
      average_val[i].append(x_edge_h)
      average_val[i].append(y_edge_h)

  #recalculate gradient and y_interc from max and min xy
  average_val=recalc_gy(average_val[0:no_cl])

  edge_sig=[[] for i in range(no_sl)]
  edge_considrd=[]
  #minimum line length requirementp
  for i in range(no_cl):
    if line_length(average_val[i])<linemin_tol:	#50 for take1,2,3, 20 for take4
      edge_considrd.append(i)

  #assigning table sides
  id_find0=find_sigedge(average_val[0:no_cl],edge_considrd,0,0)
  edge_sig[1].append(average_val[id_find0])	#table left
  edge_considrd.append(id_find0)

  id_find0=find_sigedge(average_val[0:no_cl],edge_considrd,1,0)
  edge_sig[0].append(average_val[id_find0])	#table top
  edge_considrd.append(id_find0)
  
  id_find0=find_sigedge(average_val[0:no_cl],edge_considrd,1,1)
  edge_sig[2].append(average_val[id_find0])	#table bottom
  edge_considrd.append(id_find0)

  #find parallel from remaining lines
  id_find0, id_find1=find_parallel(average_val[0:no_cl],edge_considrd,parll1_tol)
  edge_considrd.append(id_find0)
  edge_considrd.append(id_find1)
  id_find2, id_find3=find_parallel2(average_val[0:no_cl],edge_considrd,parll2_tol)
  edge_considrd.append(id_find2)
  edge_considrd.append(id_find3)
  #find intersect and create quadrilateral to represent plane
  average_val[id_find0],average_val[id_find2]=extend_planelines(average_val[id_find0],average_val[id_find2])
  average_val[id_find0],average_val[id_find3]=extend_planelines(average_val[id_find0],average_val[id_find3])
  average_val[id_find1],average_val[id_find2]=extend_planelines(average_val[id_find1],average_val[id_find2])
  average_val[id_find1],average_val[id_find3]=extend_planelines(average_val[id_find1],average_val[id_find3])
  
  #find length of each side of quadrilateral
  l_line0=line_length(average_val[id_find0])
  l_line1=line_length(average_val[id_find1])
  l_line2=line_length(average_val[id_find2])
  l_line3=line_length(average_val[id_find3])

  #assign sides of quadrilateral
  avg_length0=(l_line0+l_line1)/2
  avg_length1=(l_line2+l_line3)/2
  if avg_length0>avg_length1:
    assign_quadrl(l_line2,l_line3,average_val,id_find0,id_find1,id_find2,id_find3,edge_sig,edge_considrd)
  else:
    assign_quadrl(l_line0,l_line1,average_val,id_find2,id_find3,id_find0,id_find1,edge_sig,edge_considrd)
  
  #find mid-points of sides of quadrilateral
  mp_quadside,mp_plane=find_midquad(edge_sig[3:7])

  for i in range(4):
    x1plt=int(edge_sig[i+3][0][2])
    y1plt=int(edge_sig[i+3][0][3])
    x2plt=int(edge_sig[i+3][0][4])
    y2plt=int(edge_sig[i+3][0][5])
    cv2.line(img,(x1plt,y1plt),(x2plt,y2plt),(0,255,0),2)
  
  plt.imshow(img),plt.show()
  #cv2.imwrite('houghlines5.jpg',img)

  with open(coorddata,'w') as f:
    f.write(str(mp_plane))
    f.write('\n')
    f.write(str(mp_quadside))

#thres_val = take1(L:87,R:104), take2(L:87,R:92), take3(L:70,R:75), take4(L:77,R:78)
#interc_tol = 50 - take1,2,3, 100 - take4
#parll1_tol = take1:10, take2,3,4:20
#parll2_tol = take1:45, take2(L/R):45/20, take3:20, take4(L/R):35/45
#linemine_tol = 50 for take1,2,3, 20 for take4
#find_edges(image_name, coorddata, thres_val, interc_tol, parll1_tol, parll2_tol, linemin_tol)

#Main Programme here~~~~~~~~~~~~~~~~~
#Take1
print ("Take1")
itc_t=50
p1_t=10
p2_t=45
lmin_t=50
img_name = 'take_1_left.png'
cdata = 'take1l_test.txt'
t_v=87
find_edges(img_name, cdata, t_v, itc_t, p1_t, p2_t, lmin_t)
img_name = 'take_1_right.png'
cdata = 'take1r_test.txt'
t_v=104
find_edges(img_name, cdata, t_v, itc_t, p1_t, p2_t, lmin_t)

#Take2
print ("Take2")
itc_t=50
p1_t=20
p2_t=45
lmin_t=50
img_name = 'take_2_left.png'
cdata = 'take2l_test.txt'
t_v=87
find_edges(img_name, cdata, t_v, itc_t, p1_t, p2_t, lmin_t)
img_name = 'take_2_right.png'
cdata = 'take2r_test.txt'
p2_t=20
t_v=92
find_edges(img_name, cdata, t_v, itc_t, p1_t, p2_t, lmin_t)

#Take3
print ("Take3")
itc_t=50
p1_t=20
p2_t=20
lmin_t=50
img_name = 'take_3_left.png'
cdata = 'take3l_test.txt'
t_v=70
find_edges(img_name, cdata, t_v, itc_t, p1_t, p2_t, lmin_t)
img_name = 'take_3_right.png'
cdata = 'take3r_test.txt'
t_v=75
find_edges(img_name, cdata, t_v, itc_t, p1_t, p2_t, lmin_t)

#Take4
print ("Take4")
itc_t=100
p1_t=20
p2_t=35
lmin_t=20
img_name = 'take_4_left.png'
cdata = 'take4l_test.txt'
t_v=77
find_edges(img_name, cdata, t_v, itc_t, p1_t, p2_t, lmin_t)
img_name = 'take_4_right.png'
cdata = 'take4r_test.txt'
p2_t=45
t_v=78
find_edges(img_name, cdata, t_v, itc_t, p1_t, p2_t, lmin_t)




