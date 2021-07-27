from math import *
import numpy as np  

d1 = 0.1;
d2 = 0.15;

def fk(theta1, theta2):
	theta = theta1 + theta2;
	x=d1*cos(theta1) + d2*cos(theta);
	y=d1*sin(theta1) + d2*sin(theta);
	eep = (x, y, theta);
	return eep

def jacobian(theta1, theta2):
	theta = theta1 + theta2;
	J=[[-d1*sin(theta1)-d2*sin(theta), -d2*sin(theta)],
	   [d1*cos(theta1)+d2*cos(theta), d2*cos(theta)],
	   [1, 1]];
	return J

theta1 = 2.08781;
theta2 = -2.0005717;
delta1 = -0.003;
delta2 = 0.002;

print(fk(theta1, theta2))
#print(fk(theta1+delta1, theta2+delta2))
#print(jacobian(theta1,theta2))
#print(fk(theta1,theta2)+np.dot(jacobian(theta1,theta2), [delta1, delta2]))
