## Simple RRT* path planning

#Learning point: numpy arrays have to be RECTANGLAR

from math import *

found_goal=False
k=1
nodes=1
xr=np.array([x_start])
yr=np.array([y_start])
path_dist=np.zeros(1)
actual_path=[[0]]
counter=1	#0-goal/1-random point

def calc_new_point(x0,y0,x1,y1):
  theta=np.arctan((y0-y1)/(x0-x1))
  if (x0-x1)<=0:
    theta+=np.pi
  xnew=cos(theta)+x1
  ynew=sin(theta)+y1
  return xnew,ynew

def line_collide(x0,y0,k_lc,min_dist):	#check if line collides
  line_split=200
  line_collide=np.zeros(line_split)
  if min_dist<=1:
    x_step=(x0-xr[k_lc])/line_split
    y_step=(y0-yr[k_lc])/line_split
    for i3 in range(line_split):
      line_collide[i3]=env.check_collision(xr[k_lc]+i3*x_step,yr[k_lc]+i3*y_step)
    if True in line_collide:
      return True
    else:
      return False
  else:
    x0,y0=calc_new_point(x0,y0,xr[k_lc],yr[k_lc])
    x_step=(x0-xr[k_lc])/line_split
    y_step=(y0-yr[k_lc])/line_split
    for i3 in range(line_split):
      line_collide[i3]=env.check_collision(xr[k_lc]+i3*x_step,yr[k_lc]+i3*y_step)
    if True in line_collide:
      return True
    else:
      return False

found_goal=False
k=1
nodes=1
xr=np.array([x_start])
yr=np.array([y_start])
path_dist=np.zeros(1)
actual_path=[[0]]
counter=1	#0-goal/1-random point

while found_goal==False:
  if counter==0:
    x_sample=x_goal
    y_sample=y_goal
  elif counter==1:
    x_sample=np.random.rand()*10
    y_sample=np.random.rand()*6
  collide=env.check_collision(x_sample,y_sample)
  if not collide:
    for k in range(nodes):
      dist=sqrt((x_sample-xr[k])**2+(y_sample-yr[k])**2)
      if k==0 or dist<min_dist:
        min_dist=dist
        min_k=k
    if line_collide(x_sample,y_sample,min_k,min_dist)==False:
      print("Tree growing~")
      if min_dist>1:
        x_sample,y_sample=calc_new_point(x_sample,y_sample,xr[min_k],yr[min_k])
        path_dist=np.append(path_dist,[path_dist[min_k]+1])
      else:
        path_dist=np.append(path_dist,[path_dist[min_k]+min_dist])
        if counter==0:
          print("Found goal!")
          found_goal=True
      pl.plot([x_sample], [y_sample], "bo", markersize = 1)
      pl.plot([x_sample,xr[min_k]],[y_sample,yr[min_k]],'b',linewidth = 0.5)
      xr=np.append(xr,[x_sample])
      yr=np.append(yr,[y_sample])
      actual_path.append(actual_path[min_k][:])
      actual_path[nodes].append(nodes)
      nodes+=1
  if counter==0:
    counter=1
  elif counter==1:
    counter=0

for k in range(1,len(actual_path[nodes-1])):
  node0=actual_path[nodes-1][k-1]
  node1=actual_path[nodes-1][k]
  pl.plot([xr[node1]], [yr[node1]], "go", markersize = 2)
  pl.plot([xr[node0],xr[node1]],[yr[node0],yr[node1]],'g',linewidth = 1.5)




