## Time parameterization

from math import *

vmax=[0.2,0.2]
amax=[0.05,0.05]
sdot=[0.0,0.0]
s2dot=[0.0,0.0]

def time_par(x_in,y_in,ap_in):
  total_time=0.0
  test_time=[0.0,0.0]
  for k in range(1,len(ap_in)):
    node0=ap_in[k-1]
    node1=ap_in[k]
    sdot[0]=vmax[0]/abs(x_in[node1]-x_in[node0])
    s2dot[0]=amax[0]/abs(x_in[node1]-x_in[node0])
    sdot[1]=vmax[1]/abs(y_in[node1]-y_in[node0])
    s2dot[1]=amax[1]/abs(y_in[node1]-y_in[node0])
    for s in range(2):
      if sdot[s]>=sqrt(s2dot[s]):
        test_time[s]=2/sqrt(s2dot[s])
      else:
        test_time[s]=1/sdot[s]+sdot[s]/s2dot[s]
    total_time+=max(test_time)
  return total_time

# For PRM
print time_par(xp,yp,actual_path[n_points+1][:])

# For RRT
print time_par(xr,yr,actual_path[nodes-1][:])

# For Post-processed Path
print time_par(xrr,yrr,path_shortened)
