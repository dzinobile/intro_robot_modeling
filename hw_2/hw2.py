import sympy as sp
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
d1 = 183.3
d2 = 737.31
d3 = 387.8
d4 = 95.5
d5 = 115.5
d6 = 76.8
alpha = sp.pi/2


thetai = sp.Symbol('theta_i')
theta1 = sp.Symbol('theta_1')
theta2 = sp.Symbol('theta_2')
theta3 = sp.Symbol('theta_3')
theta4 = sp.Symbol('theta_4')
theta5 = sp.Symbol('theta_5')
theta6 = sp.Symbol('theta_6')
t = sp.Symbol('t')
ai = sp.Symbol('a_i')
di = sp.Symbol('d_i')
alphai = sp.Symbol('alpha_i')
pi = sp.Symbol('pi')


dh_table = {
    "": ["to 1", "to 2", "to 3", "to 4", "to 5", "to 6"],
    thetai: [theta1, pi/2 + theta2, theta3, pi/2 + theta4, theta5, theta6],
    ai: [0, d2, d3, 0, 0, 0],
    di: [d1, 0, 0, -d4, d5, 76.8],
    alphai: [alpha, 0, 0, alpha, -alpha, 0]

}

df = pd.DataFrame(dh_table)
print(df)
A1tz = sp.Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,d1],[0,0,0,1]])
A1rz = sp.Matrix([[sp.cos(theta1),-sp.sin(theta1),0,0],[sp.sin(theta1),sp.cos(theta1),0,0],[0,0,1,0],[0,0,0,1]])
A1rx = sp.Matrix([[1,0,0,0],[0,sp.cos(alpha),-sp.sin(alpha),0],[0,sp.sin(alpha),sp.cos(alpha),0],[0,0,0,1]])
A1 = A1tz*A1rz*A1rx

A2ty = sp.Matrix([[1,0,0,0],[0,1,0,d2],[0,0,1,0],[0,0,0,1]])
A2rz = sp.Matrix([[sp.cos((sp.pi/2)+theta2),-sp.sin((sp.pi/2)+theta2),0,0],[sp.sin((sp.pi/2)+theta2),sp.cos((sp.pi/2)+theta2),0,0],[0,0,1,0],[0,0,0,1]])
A2 = A2ty*A2rz
A3tx = sp.Matrix([[1,0,0,-d3],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
A3rz = sp.Matrix([[sp.cos(theta3),-sp.sin(theta3),0,0],[sp.sin(theta3),sp.cos(theta3),0,0],[0,0,1,0],[0,0,0,1]])
A3 = A3tx*A3rz
A4tz = sp.Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,d4],[0,0,0,1]])
A4rz = sp.Matrix([[sp.cos((sp.pi/2)+theta4),-sp.sin((sp.pi/2)+theta4),0,0],[sp.sin((sp.pi/2)+theta4),sp.cos((sp.pi/2)+theta4),0,0],[0,0,1,0],[0,0,0,1]])
A4rx = sp.Matrix([[1,0,0,0],[0,sp.cos(alpha),-sp.sin(alpha),0],[0,sp.sin(alpha),sp.cos(alpha),0],[0,0,0,1]])
A4 = A4tz*A4rz*A4rx
A5tz = sp.Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,d5],[0,0,0,1]])
A5rz = sp.Matrix([[sp.cos(theta5),-sp.sin(theta5),0,0],[sp.sin(theta5),sp.cos(theta5),0,0],[0,0,1,0],[0,0,0,1]])
A5rx = sp.Matrix([[1,0,0,0],[0,sp.cos(-alpha),-sp.sin(-alpha),0],[0,sp.sin(-alpha),sp.cos(-alpha),0],[0,0,0,1]])
A5 = A5tz*A5rz*A5rx
A6tz = sp.Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,d6],[0,0,0,1]])
A6rz = sp.Matrix([[sp.cos(theta6),-sp.sin(theta6),0,0],[sp.sin(theta5),sp.cos(theta6),0,0],[0,0,1,0],[0,0,0,1]])
A6 = A6tz*A6rz
A = A1*A2*A3*A4*A5*A6
O = sp.zeros(3,7)
a_array = [1,A1,A2,A3,A4,A5,A6]
product = sp.eye(4)
for i in range(1,7):
    product = product*a_array[i]
    O[:,i] = sp.Matrix([
        [(product)[0,3]],
        [(product)[1,3]],
        [(product)[2,3]]])




z = sp.zeros(3,7)
z[:,0] = sp.Matrix([[0],[0],[1]])
for i in range(1,7):
    z[:,i] = sp.Matrix([
        [(a_array[i])[0,2]],
        [(a_array[i])[1,2]],
        [(a_array[i])[2,2]]])

j = sp.zeros(6,6)
for i in range(0,6):
    j[:,i] = sp.Matrix([[(z.col(i)).cross((O.col(6))-(O.col(i)))],[z.col(i)]])

increment = 0.1
duration = 20
arraysize = int(duration/increment)
time = np.arange(0, duration, increment)
ee_x_initial = 0
ee_y_initial = -172.3-45
ee_z_initial = d1+d2+d3+d5

path1_x_final = -50
path1_y_final = ee_y_initial - 10
path1_z_final = 1350

path1_distance = sp.sqrt(((ee_x_initial-path1_x_final)**2)+((ee_y_initial-path1_y_final)**2)+(ee_z_initial-path1_z_final)**2)
path2_distance = 50*sp.pi
path3_distance = 50
path4_distance = 100
path5_distance = 50
path6_distance = path1_distance
total_distance = path1_distance+path2_distance+path3_distance+path4_distance+path5_distance+path6_distance

path1_time = (path1_distance/total_distance)*duration
path2_time = (path2_distance/total_distance)*duration
path3_time = (path3_distance/total_distance)*duration
path4_time = (path4_distance/total_distance)*duration
path5_time = (path5_distance/total_distance)*duration
path6_time = (path6_distance/total_distance)*duration

path1_start = 0
path2_start = path1_time.round(1)
path3_start = (path2_start+path2_time).round(1)
path4_start = (path3_start+path3_time).round(1)
path5_start = (path4_start+path4_time).round(1)
path6_start = (path5_start+path5_time).round(1)




path1_xdot = (path1_x_final-ee_x_initial)/path1_time
path1_ydot = (path1_y_final-ee_y_initial)/path1_time
path1_zdot = (path1_z_final-ee_z_initial)/path1_time

path2_x = path1_x_final+(50*sp.cos((sp.pi/path2_time)*t))
path2_y = path1_y_final
path2_z = path1_z_final+(50*sp.sin((sp.pi/path2_time)*t))
path2_xdot = sp.diff(path2_x,t)
path2_ydot = sp.diff(path2_y,t)
path2_zdot = sp.diff(path2_z,t)

path3_xdot = 0
path3_ydot = 0
path3_zdot = -50/path3_time

path4_xdot = -100/path4_time
path4_ydot = 0
path4_zdot = 0

path5_xdot = 0
path5_ydot = 0
path5_zdot = 50/path5_time

path6_xdot = -path1_xdot
path6_ydot = -path1_ydot
path6_zdot = -path1_zdot

XDOT = sp.Matrix([
    [path1_xdot, path2_xdot, path3_xdot, path4_xdot, path5_xdot, path6_xdot],
    [path1_ydot, path2_ydot, path3_ydot, path4_ydot, path5_ydot, path6_ydot],
    [path1_zdot, path2_zdot, path3_zdot, path4_zdot, path5_zdot, path6_zdot],
    [0,0,0,0,0,0],
    [0,0,0,0,0,0],
    [0,0,0,0,0,0]])

# XDOT_path1 = sp.Matrix([[path1_xdot],[path1_ydot],[path1_zdot],[0],[0],[0]])
# XDOT_path2 = sp.Matrix([[path2_xdot],[path2_ydot],[path2_zdot],[0],[0],[0]])
# XDOT_path3 = sp.Matrix([[path3_xdot],[path3_ydot],[path3_zdot],[0],[0],[0]])
# XDOT_path4 = sp.Matrix([[path4_xdot],[path4_ydot],[path4_zdot],[0],[0],[0]])
# XDOT_path5 = sp.Matrix([[path5_xdot],[path5_ydot],[path5_zdot],[0],[0],[0]])
# XDOT_path6 = sp.Matrix([[path6_xdot],[path6_ydot],[path6_zdot],[0],[0],[0]])





path1_arraysize = int(path1_time/increment)
path2_arraysize = int(path2_time/increment)
path3_arraysize = int(path3_time/increment)
path4_arraysize = int(path4_time/increment)
path5_arraysize = int(path5_time/increment)
path6_arraysize = int(path6_time/increment)


jinv_initial = (j.subs([
    (theta1,0),
    (theta2,0),
    (theta3,0),
    (theta4,0),
    (theta5,0),
    (theta6,0)])).pinv()

thetadot_initial = jinv_initial*XDOT[:,0]


path2_timearray = np.arange(path2_start, path2_start+path2_time, increment)


THETA_path1 = sp.zeros(6,path1_arraysize)


THETADOT_path1 = sp.zeros(6,path1_arraysize)

for i in range(0,6):
    THETADOT_path1[i,0] = thetadot_initial[i]











#PATH1

for n in range(1,path1_arraysize):
    for i in range(0,6):
        THETA_path1[i,n] = (THETA_path1[i,n-1]+((THETADOT_path1[i,n-1])*increment)).round(5)
    jsubs = j.subs([
        (theta1,THETA_path1[0,n]),
        (theta2,THETA_path1[1,n]),
        (theta3,THETA_path1[2,n]),
        (theta4,THETA_path1[3,n]),
        (theta5,THETA_path1[4,n]),
        (theta6,THETA_path1[5,n])
    ])
    jinv = jsubs.pinv()
    thetadot = jinv*XDOT[:,0]
    for i in range(0,6):
        THETADOT_path1[i,n] = thetadot[i]






#PATH2
THETA_path2 = sp.zeros(6,path2_arraysize)
THETADOT_path2 = sp.zeros(6,path2_arraysize)
for i in range(0,6):
    THETA_path2[i,0] = THETA_path1[i,-1]
    THETADOT_path2[i,0] = THETADOT_path1[i,-1]
    
for n in range(1,path2_arraysize):
    for i in range(0,6):
        THETA_path2[i,n] = (THETA_path2[i,n-1]+((THETADOT_path2[i,n-1])*increment)).round(5)
    jsubs = j.subs([
        (theta1,THETA_path2[0,n]),
        (theta2,THETA_path2[1,n]),
        (theta3,THETA_path2[2,n]),
        (theta4,THETA_path2[3,n]),
        (theta5,THETA_path2[4,n]),
        (theta6,THETA_path2[5,n])])
    jinv = jsubs.pinv()
    thetadot = jinv*((XDOT[:,1]).subs([(t,path2_timearray[n])]))
    for i in range(0,6):
        THETADOT_path2[i,n] = thetadot[i]


#PATH3
THETA_path3 = sp.zeros(6,path3_arraysize)
THETADOT_path3 = sp.zeros(6,path3_arraysize)

for i in range(0,6):
    THETA_path3[i,0] = THETA_path2[i,-1]
    THETADOT_path3[i,0] = THETADOT_path2[i,-1]

for n in range(1,path3_arraysize):
    for i in range(0,6):
        THETA_path3[i,n] = (THETA_path3[i,n-1]+((THETADOT_path3[i,n-1])*increment)).round(5)
    jsubs = j.subs([
        (theta1,THETA_path3[0,n]),
        (theta2,THETA_path3[1,n]),
        (theta3,THETA_path3[2,n]),
        (theta4,THETA_path3[3,n]),
        (theta5,THETA_path3[4,n]),
        (theta6,THETA_path3[5,n])])
    jinv = jsubs.pinv()
    thetadot = jinv*(XDOT[:,2])
    for i in range(0,6):
        THETADOT_path3[i,n] = thetadot[i]

#PATH4
THETA_path4 = sp.zeros(6,path4_arraysize)
THETADOT_path4 = sp.zeros(6,path4_arraysize)

for i in range(0,6):
    THETA_path4[i,0] = THETA_path3[i,-1]
    THETADOT_path4[i,0] = THETADOT_path3[i,-1]

for n in range(1,path4_arraysize):
    for i in range(0,6):
        THETA_path4[i,n] = (THETA_path4[i,n-1]+((THETADOT_path4[i,n-1])*increment)).round(5)
    jsubs = j.subs([
        (theta1,THETA_path4[0,n]),
        (theta2,THETA_path4[1,n]),
        (theta3,THETA_path4[2,n]),
        (theta4,THETA_path4[3,n]),
        (theta5,THETA_path4[4,n]),
        (theta6,THETA_path4[5,n])])
    jinv = jsubs.pinv()
    thetadot = jinv*(XDOT[:,3])
    for i in range(0,6):
        THETADOT_path4[i,n] = thetadot[i]

#PATH 5
THETA_path5 = sp.zeros(6,path5_arraysize)
THETADOT_path5 = sp.zeros(6,path5_arraysize)

for i in range(0,6):
    THETA_path5[i,0] = THETA_path4[i,-1]
    THETADOT_path5[i,0] = THETADOT_path4[i,-1]

for n in range(1,path5_arraysize):
    for i in range(0,6):
        THETA_path5[i,n] = (THETA_path5[i,n-1]+((THETADOT_path5[i,n-1])*increment)).round(5)
    jsubs = j.subs([
        (theta1,THETA_path5[0,n]),
        (theta2,THETA_path5[1,n]),
        (theta3,THETA_path5[2,n]),
        (theta4,THETA_path5[3,n]),
        (theta5,THETA_path5[4,n]),
        (theta6,THETA_path5[5,n])])
    jinv = jsubs.pinv()
    thetadot = jinv*(XDOT[:,4])
    for i in range(0,6):
        THETADOT_path5[i,n] = thetadot[i]


#PATH 6
THETA_path6 = sp.zeros(6,path6_arraysize)
THETADOT_path6 = sp.zeros(6,path6_arraysize)

for i in range(0,6):
    THETA_path6[i,0] = THETA_path5[i,-1]
    THETADOT_path6[i,0] = THETADOT_path5[i,-1]

for n in range(1,path6_arraysize):
    for i in range(0,6):
        THETA_path6[i,n] = (THETA_path6[i,n-1]+((THETADOT_path6[i,n-1])*increment)).round(5)
    jsubs = j.subs([
        (theta1,THETA_path6[0,n]),
        (theta2,THETA_path6[1,n]),
        (theta3,THETA_path6[2,n]),
        (theta4,THETA_path6[3,n]),
        (theta5,THETA_path6[4,n]),
        (theta6,THETA_path6[5,n])])
    jinv = jsubs.pinv()
    thetadot = jinv*(XDOT[:,5])
    for i in range(0,6):
        THETADOT_path6[i,n] = thetadot[i]

THETA_path6


theta1_all = [0]*arraysize

for i in range(0,path1_arraysize):
    theta1_all[i] = THETA_path1[0,i]

n = path1_arraysize
for i in range(n,n+path2_arraysize):
    theta1_all[i] = THETA_path2[0,i-n]

n = n+path2_arraysize
for i in range(n,n+path3_arraysize):
    theta1_all[i] = THETA_path3[0,i-n]

n = n+path3_arraysize
for i in range(n,n+path4_arraysize):
    theta1_all[i] = THETA_path4[0,i-n]

n=n+path4_arraysize
for i in range(n,n+path5_arraysize):
    theta1_all[i] = THETA_path5[0,i-n]

n = n+path5_arraysize
for i in range(n,n+path6_arraysize):
    theta1_all[i] = THETA_path6[0,i-n]



link1x = [0]*arraysize
link1y = [0]*arraysize
link1z = [0]*arraysize

for i in range(0,arraysize):
    link1x[i] = 172.3*sp.sin(theta1_all[i])
    link1y[i] = -172.3 + (172.3*sp.cos(theta1_all[i]))
    link1z[i] = 183.3


# i = 0
# while i < path1_arraysize:
#     plt.clf()
#     plt.plot([0, link1x[i]],[0, link2z2array[i]])
#     plt.axis('square')
#     plt.grid(True)
#     plt.xlim([-500,500])
#     plt.ylim([-500,500])
#     plt.gca().set_aspect('equal', adjustable='box')
#     i = i+1
#     plt.pause(0.05)   

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')



# Plot 
for i in range(0,path1_arraysize):
   
    ax.scatter([0,link1x[i]], [0,link1y[i]], [0,link1z[i]], cmap='viridis')
    ax.plot([0,link1x[i]], [0, link1y[i]], [0, link1z[i]])
    
    
   


# Set labels
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

# Show plot
plt.show()


