## Post-processing (shortcutting)

from math import *

def line_collide2(x0,y0,x1,y1):
  line_split=1000
  line_col=np.zeros(line_split)
  x_step=(x1-x0)/line_split
  y_step=(y1-y0)/line_split
  for i3 in range(line_split):
    line_col[i3]=env.check_collision(x0+i3*x_step,y0+i3*y_step)
  if True in line_col:
    return True
  else:
    return False

def path_print(x_in2,y_in2,ap_in2):
  for pp in range(1,len(ap_in2)):
    node0=ap_in2[pp-1]
    node1=ap_in2[pp]
    pl.plot([x_in2[node1]], [y_in2[node1]], "ro", markersize = 5)
    pl.plot([x_in2[node0],x_in2[node1]],[y_in2[node0],y_in2[node1]],'y',linewidth = 2.5)

xrr=[]
yrr=[]
path_shortened=[]

def shortcutter(x_in,y_in,ap_in,noofpts):
  global xrr,yrr,path_shortened
  path_short=ap_in
  print path_short
  len_path=len(path_short)
  n_shortcuts=0
  remove_section=[0,0]
  print("start loop")
  while n_shortcuts<50 and len_path>5:
    path_section=floor(np.random.rand()*(len_path-1))+1	#choosing section of line
    path_same=True
    while path_same==True:
      path_section2=floor(np.random.rand()*(len_path-1))+1
      if path_section!=path_section2:
        path_same=False
    path_section=int(path_section)
    path_section2=int(path_section2)
    node0=path_short[path_section]
    node1=path_short[path_section-1]
    r_lineseg=np.random.rand()
    xtest0=x_in[node0]+r_lineseg*(x_in[node1]-x_in[node0])	#random point 1
    ytest0=y_in[node0]+r_lineseg*(y_in[node1]-y_in[node0])
    node0=path_short[path_section2]
    node1=path_short[path_section2-1]
    r_lineseg=np.random.rand()
    xtest1=x_in[node0]+r_lineseg*(x_in[node1]-x_in[node0])	#random point 2
    ytest1=y_in[node0]+r_lineseg*(y_in[node1]-y_in[node0])
    if line_collide2(xtest0,ytest0,xtest1,ytest1):	#collision (no shortcut)
      print("-")
      continue
    else:	#no collision, proceed with route change
      if path_section2>path_section:
        remove_section[0]=path_section	#lower section (nearer to start)
        remove_section[1]=path_section2	#higher section (nearer to goal)
      else:
        remove_section[0]=path_section2
        remove_section[1]=path_section
      for r_s in range(remove_section[0],remove_section[1]):	#remove old paths
        path_short.pop(remove_section[0])
      path_short.insert(remove_section[0],noofpts)	#add new paths
      noofpts+=1
      path_short.insert(remove_section[0],noofpts)
      noofpts+=1
      len_path=len(path_short)
      if path_section2>path_section:
        x_in=np.append(x_in,xtest1)
        y_in=np.append(y_in,ytest1)
        x_in=np.append(x_in,xtest0)
        y_in=np.append(y_in,ytest0)
      else:
        x_in=np.append(x_in,xtest0)
        y_in=np.append(y_in,ytest0)
        x_in=np.append(x_in,xtest1)
        y_in=np.append(y_in,ytest1)
      n_shortcuts+=1    
      print("no collide")
  path_print(x_in,y_in,path_short)
  print path_short
  xrr=x_in
  yrr=y_in
  path_shortened=path_short

# For PRM
shortcutter(xp,yp,actual_path[n_points+1][:],n_points+2)

# For RRT
shortcutter(xr,yr,actual_path[nodes-1][:],nodes)



