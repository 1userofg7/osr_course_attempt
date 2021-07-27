## PRM

#initialize
import numpy as np
import pylab as pl
from math import *

n_points=1000
xp=np.zeros(n_points+2)
yp=np.zeros(n_points+2)
collide=np.zeros(n_points)
path_e=np.zeros((n_points+2,n_points+2)) #check if path exists (1-yes/2-no)
path_l=np.zeros((n_points+2,n_points+2)) #path length
path=np.zeros((n_points,1)) #actual possible paths
path_temp=np.zeros(n_points)

def check_prox(x0,x1,y0,y1,ii,jj):
  global path_e
  global path_l
  line_split=200
  line_collide=np.zeros(line_split)
  dist=sqrt((x0-x1)**2+(y0-y1)**2)
  if dist<=1:	#distance check here
    x_step=(x0-x1)/line_split
    y_step=(y0-y1)/line_split
    for i3 in range(line_split):
      line_collide[i3]=env.check_collision(x1+i3*x_step,y1+i3*y_step)
    if True in line_collide:
      return
    else:
      pl.plot([x0,x1],[y0,y1],'b',linewidth = 0.2)
      path_e[ii,jj]=1
      path_e[jj,ii]=1
      path_l[ii,jj]=dist
      path_l[jj,ii]=dist


for i in range(n_points):
  xp[i]=np.random.rand()*10
  yp[i]=np.random.rand()*6
  collide[i]=env.check_collision(xp[i],yp[i])
  if collide[i]:
    pl.plot([xp[i]], [yp[i]], "ro", markersize = 1)
  else:
    pl.plot([xp[i]], [yp[i]], "bo", markersize = 1)
    check_prox(xp[i],x_start,yp[i],y_start,i,n_points)
    check_prox(xp[i],x_goal,yp[i],y_goal,i,n_points+1)
    if i>1:
      for j in range(i):
        if not collide[j]:
          check_prox(xp[i],xp[j],yp[i],yp[j],i,j)

xp[n_points]=x_start
yp[n_points]=y_start
xp[n_points+1]=x_goal
yp[n_points+1]=y_goal

## Simple A* search algo for PRM~~~~~~~~~~~~~~~~~~~~~~~

open_list=np.array([n_points+2])
close_list=np.array([n_points])
f=np.zeros(1)
g=np.zeros(1)
size=1
actual_path=[[] for z in range(n_points+2)]

for z in range(n_points+2):
  actual_path[z].append(n_points)

def node_tester(c_point,c_path): #returns next best point to be evaluated (n_point+2 if no other point)
  global f,g,actual_path
  global size
  global open_list, close_list
  close_list=np.append(close_list,c_point)
  print("casetest")
  for k in range(n_points+2):
    if k!=c_point and path_e[c_point,k]==1:	#t: not current point && path exists
      print("caseconditionyes")
      if k in close_list:	#t:in close_list
        print("case1")
        continue
      elif k in open_list:	#t:in existing open_list
        print("case2")
        h=sqrt((xp[c_point]-x_goal)**2+(yp[c_point]-y_goal)**2)	#heuristic distance
        g_sample=c_path+path_l[c_point,k]	#total path length
        f_sample=h+g_sample	#f-value
        if f_sample<f[np.where(f == k)]:	#compare current and new f-value
          actual_path[k]=actual_path[c_point][:]
          actual_path[k].append(k)
          open_list=np.delete(open_list,size-2)	#removal of old
          g=np.delete(g,size-2)
          f=np.delete(f,size-2)
          size-=1
          size_2=size
          for kk in range(size_2):	#replacement with new
            if f_sample>f[kk]:
              f=np.insert(f,kk,f_sample)
              g=np.insert(g,kk,g_sample)
              open_list=np.insert(open_list,kk,k)
              size+=1
              break
            else:
              continue
      else:
        print("case3")
        actual_path[k]=actual_path[c_point][:]
        print("line1")
        actual_path[k].append(k)
        print("line2")
        h=sqrt((xp[c_point]-x_goal)**2+(yp[c_point]-y_goal)**2)	#heuristic distance
        print("line3")
        g_sample=c_path+path_l[c_point,k]	#total path length
        print("line4")
        f_sample=h+g_sample	#f-value
        size_2=size
        for kk in range(size_2):	#placement of new open node
          if f_sample>f[kk]:
            print("line5")
            f=np.insert(f,kk,f_sample)
            print("line6")
            g=np.insert(g,kk,g_sample)
            print("line7")
            open_list=np.insert(open_list,kk,k)
            print("line8")
            size+=1
            break
          else:
            continue
    print k
    print size
  if size<=1:	#no solution possible
    return n_points+2, 0
  elif open_list[size-2]==n_points+1:	#best solution is goal found!
    return n_points+1, 0
  else:	#testing with next min f-value member
    n_min=open_list[size-2]
    new_path=g[size-2]
    open_list=np.delete(open_list,size-2)
    g=np.delete(g,size-2)
    f=np.delete(f,size-2)
    size-=1
    return n_min, new_path

g_cp=n_points	#global current point = starting point
g_path=0	#global total path = starting path of zero

while g_cp<n_points+1:
  g_cp,g_path=node_tester(g_cp,g_path)	#actual code

if g_cp==n_points+1:	#obtain results
  print("Solution found:")
  print actual_path[n_points+1][:]
elif g_cp==n_points+2:
  print("No solution found.")
else:
  print("Something wrong with algo")	

for k in range(1,len(actual_path[n_points+1])):
  node0=actual_path[n_points+1][k-1]
  node1=actual_path[n_points+1][k]
  pl.plot([xp[node0]], [yp[node0]], "go", markersize = 2)
  pl.plot([xp[node1]], [yp[node1]], "go", markersize = 2)
  pl.plot([xp[node0],xp[node1]],[yp[node0],yp[node1]],'g',linewidth = 1.5)











