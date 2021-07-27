from math import *
import numpy as np  

d1= 0.1;
d2= 0.15;
theta_des=[[0.0,0.0],[0.0,0.0]];
k1=[0.0,0.0];
k2=[0.0,0.0];

def ik(x_des,y_des):
	print(x_des**2+y_des**2-d1**2-d2**2)/(2*d1*d2)
	theta_des[1][0]=np.arccos((x_des**2+y_des**2-d1**2-d2**2)/(2*d1*d2));
	theta_des[1][1]=-np.arccos((x_des**2+y_des**2-d1**2-d2**2)/(2*d1*d2));
	k1[0]=d1+d2*cos(theta_des[1][0]);
	k1[1]=d1+d2*cos(theta_des[1][1]);
	k2[0]=d2*sin(theta_des[1][0]);
	k2[1]=d2*sin(theta_des[1][1]);
	theta_des[0][0]=np.arctan2(y_des,x_des)-np.arctan2(k2[0],k1[0]);
	theta_des[0][1]=np.arctan2(y_des,x_des)-np.arctan2(k2[1],k1[1]);

	return theta_des

ik(0.1,0.1)
print theta_des

ik(0.1,0.2)
print theta_des

