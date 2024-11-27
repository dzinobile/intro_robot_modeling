import sympy as sp
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
from sympy import pprint
#INITIALIZE VARIABLES

d1 = 183.3 #joint 1 height [mm]
d2 = 737.31 #joint 2 radius [mm]
d3 = 387.8 #joint 3 radius [mm]
d4 = 95.5 #joint 4 radius [mm]
d5 = 115.5 #joint 5 radius [mm]
d6 = 76.8+45 #end effector offset
q = sp.pi/2 #standard 90 degree angle for ease of coding


#INITIALIZE SYMBOLS
thetai = sp.Symbol('theta_i')
t = sp.Symbol('t')
ai = sp.Symbol('a_i')
di = sp.Symbol('d_i')
alphai = sp.Symbol('alpha_i')
pi = sp.Symbol('pi')
#CREATE DH TABLE ROWS

thetas = sp.symbols('theta_1:7')
dh_theta = [thetas[0], thetas[1]+q, thetas[2], thetas[3]+q, thetas[4], thetas[5]]
dh_a = [0, d2, d3, 0, 0, 0]
dh_d = [d1, 0, 0, d4, d5, d6]
dh_alpha = [q, 0, 0, q, -q, 0]
#CREATE DH TABLE

dh_table = {
    "": ["to 1", "to 2", "to 3", "to 4", "to 5", "to 6"],
    thetai: [dh_theta[0],dh_theta[1],dh_theta[2],dh_theta[3],dh_theta[4],dh_theta[5]],
    ai: [dh_a[0],dh_a[1],dh_a[2],dh_a[3],dh_a[4],dh_a[5]],
    di: [dh_d[0],dh_d[1],dh_d[2],dh_d[3],dh_d[4],dh_d[5]],
    alphai: [dh_alpha[0],dh_alpha[1],dh_alpha[2],dh_alpha[3],dh_alpha[4],dh_alpha[5]]

}

df = pd.DataFrame(dh_table)
df
#CREATE TRANSFORMATION MATRICES

#Array of all transformation matrices
A_array = [sp.zeros(4,4)]*6
for i in range(0,6):
    A_array[i] = sp.Matrix([
        [sp.cos(dh_theta[i]),-sp.sin(dh_theta[i])*sp.cos(dh_alpha[i]),sp.sin(dh_theta[i])*sp.sin(dh_alpha[i]),dh_a[i]*sp.cos(dh_theta[i])],
        [sp.sin(dh_theta[i]),sp.cos(dh_theta[i])*sp.cos(dh_alpha[i]),-sp.cos(dh_theta[i])*sp.sin(dh_alpha[i]),dh_a[i]*sp.sin(dh_theta[i])],
        [0,sp.sin(dh_alpha[i]),sp.cos(dh_alpha[i]),dh_d[i]],
        [0,0,0,1]
    ])

#Final transformation matrix A1*A2*A3*A4*A5*A6
A_final = A_array[0]
for i in range(1,6):
    A_final = A_final*A_array[i]

#Array of transformation matrix products [A1, A1*A2, A1*A2*A3,] etc
A_products = [A_array[0]]*6
for i in range(1,6):
    A_products[i] = A_products[i-1]*A_array[i]


#MATRIX SUBSTITUTION FUNCTIONS

#Substitutes theta values into given matrix and returns matrix 
def subs_function(matrix,sub_values):
    result = matrix.subs([
        (thetas[0],sub_values[0]),
        (thetas[1],sub_values[1]),
        (thetas[2],sub_values[2]),
        (thetas[3],sub_values[3]),
        (thetas[4],sub_values[4]),
        (thetas[5],sub_values[5])     
    ])
    return result

#Returns position vector of given transformation matrix and theta values
def subs_position_vector(matrix,sub_values):
    msubs = matrix.subs([
        (thetas[0],sub_values[0]),
        (thetas[1],sub_values[1]),
        (thetas[2],sub_values[2]),
        (thetas[3],sub_values[3]),
        (thetas[4],sub_values[4]),
        (thetas[5],sub_values[5])   
    ])
    result = sp.Matrix([
        [msubs[0,3]],
        [msubs[1,3]],
        [msubs[2,3]],
        [0],
        [0],
        [0]
    ])
    return result



#VALIDATE TRANSFORMATION MATRIX

#5 different sets of theta values for validation
val_matrix = sp.Matrix([
    [0, q, 0, 0, 0],
    [0, 0, q, 0, 0],
    [0, 0, 0, q, 0],
    [0, 0, 0, 0, q],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]
])

#Array of resulting matrices
A_vals = [sp.zeros(4,4)]*5
for i in range(0,5):
    A_vals[i] = subs_function(A_final,val_matrix[:,i])


pos_vals = ["start position","first joint rotated 90 degrees","shoulder rotated 90 degrees","elbow rotated 90 degrees","wrist rotated 90 degrees"]
print("Transformation matrix validation")
for i in range(0,5):
    print("---------------")
    print(pos_vals[i])
    print(
        "x = ",((A_vals[i])[0,3]).round(2),#get x value from matrix i in A_vals array
        "y = ",((A_vals[i])[1,3]).round(2),#get y value from matrix i in A_vals array
        "z = ",((A_vals[i])[2,3]).round(2))#get z value from matrix i in A_vals array



#CREATE JACOBIAN


thetas_vector = sp.Matrix([thetas[0],thetas[1],thetas[2],thetas[3],thetas[4],thetas[5]])

# P = sp.Matrix([A_final[0,3],A_final[1,3],A_final[2,3]])

# Jv = P.jacobian(thetas_vector) #create 3x6 Jv matrix

Z = sp.zeros(3,7) #initialize 3x6 Jw matrix of zeros
Z[:,0] = sp.Matrix([0,0,1])

#replace values in Z with correct values from matrices in A_products array
for c in range(1,7):
    for r in range(0,3):
        Z[r,c] = (A_products[c-1])[r,2]

O = sp.zeros(3,7)
O[:,0] = sp.Matrix([0,0,0])
for c in range(1,7):
    for r in range(0,3):
        O[r,c] = (A_products[c-1])[r,3]


Jv = sp.zeros(3,6) #initialize 6x6 matrix of zeros
for c in range(0,6):
    Jv[:,c] = Z[:,c].cross(O[:,-1]-O[:,c])

#replace values with values from Jv and Jw
J= sp.zeros(6,6)
for c in range(0,6):
    for r in range(0,3):
        J[r,c] = Jv[r,c]
        J[r+3,c] = Z[r,c]





#PATH INPUTS

#initialize inputs
increment = 1 #time increment [s]
duration = 200 #total time [s]
arraysize = int(duration/increment) 
time = np.arange(0, duration, increment) #time array

theta_initial = sp.Matrix([0,3.14/4,3.14/4,0,3.14/4,0]) #Initial theta values (chosen at random)
ee_A_final = subs_position_vector(A_final,theta_initial) #Initial x,y,z position of end effector based on initial theta values

#initial end effector posiitons with respect to base frame [mm]
ee_x_initial = ee_A_final[0] 
ee_y_initial = ee_A_final[1]
ee_z_initial = ee_A_final[2]


#calculate distances traveled during paths
path1_distance = 50*sp.pi
path2_distance = 50
path3_distance = 100
path4_distance = 50
total_distance = path1_distance+path2_distance+path3_distance+path4_distance

#calculate time taken for each path
path1_time = (path1_distance/total_distance)*duration
path2_time = (path2_distance/total_distance)*duration
path3_time = (path3_distance/total_distance)*duration
path4_time = (path4_distance/total_distance)*duration

#calculate start time of each path
path1_start = 0
path2_start = path1_time.round(1)
path3_start = (path2_start+path2_time).round(1)
path4_start = (path3_start+path3_time).round(1)




#VELOCITY PROFILES

#equation for path 1 position with respect to time
path1_x = -(50*sp.cos((sp.pi/path1_time)*t))
path1_y = 0
path1_z = (50*sp.sin((sp.pi/path1_time)*t))

#equation for path 1 velocities with repect to time
path1_xdot = sp.diff(path1_x,t)
path1_ydot = 0
path1_zdot = sp.diff(path1_z,t)

#calculate path 2 velocities
path2_xdot = 0
path2_ydot = 0
path2_zdot = -50/path2_time

#calculate path 3 velocities
path3_xdot = -100/path3_time
path3_ydot = 0
path3_zdot = 0

#calculate path 4 velocities
path4_xdot = 0
path4_ydot = 0
path4_zdot = 50/path4_time

#create xdot master matrix for all paths 
XDOT = sp.Matrix([
    [path1_xdot, path2_xdot, path3_xdot, path4_xdot],
    [path1_ydot, path2_ydot, path3_ydot, path4_ydot],
    [path1_zdot, path2_zdot, path3_zdot, path4_zdot],
    [0,0,0,0],
    [0,0,0,0],
    [0,0,0,0]])



#initialize array sizes for each path (for use in for loops)
path1_arraysize = int(path1_time/increment)
path2_arraysize = int(path2_time/increment)
path3_arraysize = int(path3_time/increment)
path4_arraysize = int(path4_time/increment)



#calculate initial inverse jacobian to obtain initial thetadot values


j_initial = (subs_function(J,theta_initial)) #initial  jacobian

if j_initial.det()<0.001:

    jinv_initial = j_initial.pinv()
else:
    jinv_initial = j_initial.inv()

thetadot_initial = jinv_initial*((XDOT[:,0]).subs([(t,0)])) #initial thetadot values














#CALCULATE ANGULAR POSITIONS AND SPEEDS

#Create theta and thetadot arrays and populate with initial values
THETA = sp.zeros(6,arraysize)
THETA[:,0] = theta_initial
THETADOT = sp.zeros(6,arraysize)
THETADOT[:,0] = thetadot_initial

#Fill theta and thetadot arrays
for c in range(1,arraysize):
    t_var = time[c]
    for r in range(0,6):
        THETA[r,c] = (THETA[r,c-1]+((THETADOT[r,c-1])*increment)).round(10)#rounded to speed up program run time
    JSUBS = subs_function(J,THETA[:,c])

    if JSUBS.det()<0.0001:
        jinv = JSUBS.pinv()
    else:
        jinv = JSUBS.inv()
    
    if t_var < path2_start:
        thetadot_temp = jinv*((XDOT[:,0]).subs([(t, t_var)]))
    elif path2_start <= t_var < path3_start:
        thetadot_temp = jinv*XDOT[:,1]
    elif path3_start <= t_var < path4_start:
        thetadot_temp = jinv*XDOT[:,2]
    else:
        thetadot_temp = jinv*XDOT[:,3]

    for r in range(0,6):
        THETADOT[r,c] = thetadot_temp[r]

    
#PLOTTING END EFFECTOR POSITION
end_effector_xyz = sp.zeros(3,arraysize)#initialize matrix of xyz positions 

#sub theta array values into final transformation matrix to get end effector xyz values
for c in range(0,arraysize):
    Asubs = subs_function(A_final,THETA[:,c])
    end_effector_xyz[0,c] = (Asubs[0,3]).round(5)
    end_effector_xyz[1,c] = (Asubs[1,3]).round(5)
    end_effector_xyz[2,c] = (Asubs[2,3]).round(5)



#PLOT END EFFECTOR PATH
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for i in range(0,arraysize):
    
    ax.scatter([end_effector_xyz[0,i]], [end_effector_xyz[1,i]], [end_effector_xyz[2,i]])


# Set labels
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')


# Show plot
plt.show()


#Rough estimate of weight of each link
m = 11 #robot mass is 11 kg

#lengths of the 8 cylinders
c1 = 173.3
c2 = 172.3
c3 = 737.31
c4 = 172.3
c5 = 387.8
c6 = 95.5
c7 = 115.5
c8 = 76.8
cpen = 45
mpl = m/(c1+c2+c3+c4+c5+c6+c7+c8) #mass per unit length of cylinder

#weights of each link
link1mass = mpl*c1
link2mass = mpl*(c2+c3)
link3mass = mpl*(c4+c5)
link4mass = mpl*c6
link5mass = mpl*c7
link6mass = mpl*c8


cm1z = (96.65)*0.001
cm2z = (183.3+(305.43*sp.cos(thetas[1])))*0.001
cm3z = (183.3+737.31*sp.cos(thetas[1])+138.67*sp.cos(thetas[1]+thetas[2]))*0.001
cm4z = (183.3+737.31*sp.cos(thetas[1])+387.8*sp.cos(thetas[1]+thetas[2]))*0.001
cm5z = (cm4z+57.75*sp.cos(thetas[1]+thetas[2]+thetas[3]))*0.001
cm6z = (cm4z+(115.5*sp.cos(thetas[1]+thetas[2]+thetas[3]))+38.4*sp.sin(thetas[4]))*0.001










#Potential energy equation as a function of thetas 1-6
PE = 9.81*(
    (link1mass*cm1z)+
    (link2mass*cm2z)+
    (link3mass*cm3z)+
    (link4mass*cm4z)+
    (link5mass*cm5z)+
    (link6mass*cm6z))


#g vector is partial derivative of PE with respect to thetas 1-6
g = sp.zeros(6,1)
for i in range(0,6):
    g[i] = sp.diff(PE,thetas[i])

#PRINT PARAMETRIC GRAVITY MATRIX
print("parametric gravity matrix")
pprint(g)













#CALCULATE REQUIRED TORQUE (NOT INCLUDING FRICTION FROM PEN)
F = sp.Matrix([0,-5,0,0,0,0])# force vector only considers normal force on pen
JT = J.transpose()#transpose of original jacobian from original DH table
torque = g - ((JT*0.001)*F)#torque equation, 0.001 is to convert original jacobian to meters
torque_array = sp.zeros(6,arraysize)
for c in range(0,arraysize):
    torque_array[:,c] = ((subs_function(torque,THETA[:,c]))).applyfunc(lambda x: round(x,5))


#CALCULATE REQUIRED TORQUE, INCLUDING FRICTION FROM THE PEN
#Find direction of travel for each path by finding unit vectors of xdot,ydot,zdot
PATH_UNIT_VECTORS = sp.zeros(3,4)
for c in range(0,4):
    for r in range(0,3):
        PATH_UNIT_VECTORS[r,c] = XDOT[r,c]/(XDOT[:3,c].norm())

friction_force = 0.3*5 #force from coefficient of friction 0.3 and normal force 5 N

#4 separate force vectors, one for each path, dependent on direction of travel
F_with_friction = sp.zeros(6,4)
for c in range(0,4):
    F_with_friction[0,c] = -friction_force*PATH_UNIT_VECTORS[0,c]
    F_with_friction[1,c] = -5
    F_with_friction[2,c] = -friction_force*PATH_UNIT_VECTORS[2,c]
    F_with_friction[3,c] = 0
    F_with_friction[4,c] = 0
    F_with_friction[5,c] = 0

#Torque equation for each joint for each path
torque_with_friction = sp.zeros(6,4)
for c in range(0,4):
    torque_with_friction[:,c] = g-((JT*0.001)*F_with_friction[:,c])

#Torque array to compensate for link weights, normal force, and friction force
torque_with_friction_array = sp.zeros(6,arraysize)
for c in range(0,arraysize):
    t_var = time[c]
    if t_var < path2_start:
        torque_with_friction_array[:,c] = ((subs_function((torque_with_friction[:,0]).subs([(t,t_var)]),THETA[:,c]))).applyfunc(lambda x: round(x,5))
    elif path2_start <= t_var < path3_start:
        torque_with_friction_array[:,c] = ((subs_function(torque_with_friction[:,1],THETA[:,c]))).applyfunc(lambda x: round(x,2))
    elif path3_start <= t_var < path4_start:
        torque_with_friction_array[:,c] = ((subs_function(torque_with_friction[:,2],THETA[:,c]))).applyfunc(lambda x: round(x,2))
    else:
        torque_with_friction_array[:,c] = ((subs_function(torque_with_friction[:,3],THETA[:,c]))).applyfunc(lambda x: round(x,2))




figure, axis = plt.subplots(2,3)

axis[0,0].plot(time, (torque_array[0,:]).transpose())
axis[0,0].set_title("Joint 1 torque")
axis[0,0].set_xlabel("Time [s]")
axis[0,0].set_ylabel("Torque [N*m]")

axis[0,1].plot(time, (torque_array[1,:]).transpose())
axis[0,1].set_title("Joint 2 torque")
axis[0,1].set_xlabel("Time [s]")
axis[0,1].set_ylabel("Torque [N*m]")

axis[0,2].plot(time, (torque_array[2,:]).transpose())
axis[0,2].set_title("Joint 3 torque")
axis[0,2].set_xlabel("Time [s]")
axis[0,2].set_ylabel("Torque [N*m]")

axis[1,0].plot(time, (torque_array[3,:]).transpose())
axis[1,0].set_title("Joint 4 torque")
axis[1,0].set_xlabel("Time [s]")
axis[1,0].set_ylabel("Torque [N*m]")


axis[1,1].plot(time, (torque_array[4,:]).transpose())
axis[1,1].set_title("Joint 5 torque")
axis[1,1].set_xlabel("Time [s]")
axis[1,1].set_ylabel("Torque [N*m]")

axis[1,2].plot(time, (torque_array[5,:]).transpose())
axis[1,2].set_title("Joint 6 torque")
axis[1,2].set_xlabel("Time [s]")
axis[1,2].set_ylabel("Torque [N*m]")


plt.tight_layout(rect=[0,0,1,0.96])
figure.suptitle("Joint Torques (friction negligible)")
plt.show()
figure, axis = plt.subplots(2,3)

axis[0,0].plot(time, (torque_with_friction_array[0,:]).transpose())
axis[0,0].set_title("Joint 1 torque")
axis[0,0].set_xlabel("Time [s]")
axis[0,0].set_ylabel("Torque [N*m]")

axis[0,1].plot(time, (torque_with_friction_array[1,:]).transpose())
axis[0,1].set_title("Joint 2 torque")
axis[0,1].set_xlabel("Time [s]")
axis[0,1].set_ylabel("Torque [N*m]")

axis[0,2].plot(time, (torque_with_friction_array[2,:]).transpose())
axis[0,2].set_title("Joint 3 torque")
axis[0,2].set_xlabel("Time [s]")
axis[0,2].set_ylabel("Torque [N*m]")

axis[1,0].plot(time, (torque_with_friction_array[3,:]).transpose())
axis[1,0].set_title("Joint 4 torque")
axis[1,0].set_xlabel("Time [s]")
axis[1,0].set_ylabel("Torque [N*m]")


axis[1,1].plot(time, (torque_with_friction_array[4,:]).transpose())
axis[1,1].set_title("Joint 5 torque")
axis[1,1].set_xlabel("Time [s]")
axis[1,1].set_ylabel("Torque [N*m]")

axis[1,2].plot(time, (torque_with_friction_array[5,:]).transpose())
axis[1,2].set_title("Joint 6 torque")
axis[1,2].set_xlabel("Time [s]")
axis[1,2].set_ylabel("Torque [N*m]")


plt.tight_layout(rect=[0,0,1,0.96])
figure.suptitle("Joint Torques (friction considered)")
plt.show()