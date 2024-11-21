
import sympy as sp
import matplotlib.pyplot as plt
import numpy as np


#Input parameters
r = 20 #radius of circle [cm]
duration = 30 #time to complete circle [s]
x_initial = 20 #initial x position of end effector [cm]
y_initial = 0 #initial y position of end effector [cm]
l1 = 15 #link 1 length [cm]
l2 = 15 #link 2 length [cm]
l3 = 2 #link 3 length [cm]
theta1_initial = 0.93 #initial angle of link 1 [rad]]
theta2_initial = -1.86 #initial angle of link 2 [rad]
theta3_initial = 0.93 #initial angle of link 3 [rad]
increment = 0.1 #time increment [s]


#Create symbols for symbolic equations
l_1 = sp.Symbol('l_1')
l_2 = sp.Symbol('l_2')
l_3 = sp.Symbol('l_3')
t = sp.Symbol('t')
C = sp.Symbol('C')
theta_1 = sp.Symbol('theta_1')
theta_2 = sp.Symbol('theta_2')
theta_3 = sp.Symbol('theta_3')

#End effector position and angle equations
x_ee = l_1*sp.cos(theta_1) + l_2*sp.cos(theta_1+theta_2) + l_3*sp.cos(theta_1+theta_2+theta_3)
y_ee = l_1*sp.sin(theta_1) + l_2*sp.sin(theta_1+theta_2) + l_3*sp.sin(theta_1+theta_2+theta_3)
theta_ee = theta_1+theta_2+theta_3

#Create end effector position vector and link angles vector
position_vector = sp.Matrix([x_ee, y_ee, theta_ee])
theta_vector = sp.Matrix([theta_1, theta_2, theta_3])
print(position_vector)
#Create jacobian and inverse jacobian 
J = position_vector.jacobian(theta_vector)
J_inv = J.inv()

#End effector x and y equations for circular path
x_c = 20*sp.cos((sp.pi/15)*t)
y_c = 20*sp.sin((sp.pi/15)*t)

#End effector x and y speed equations and initial x and y speed values
xdot_c = sp.diff(x_c, t)
ydot_c = sp.diff(y_c, t)
xdot_initial = xdot_c.subs(t, 0)
ydot_initial = ydot_c.subs(t,0)

#Find angular velocities vector
velocity_c = sp.Matrix([xdot_c, ydot_c, 0])#velociy vector for circular path with link 3 always horizontal
thetadot_c = J_inv*velocity_c #multiply by inverse jacobian to find angular velocities

#Find initial angular speeds of links 1, 2, and 3
thetadot_c = thetadot_c.subs([(l_1, l1),(l_2, l2),(l_3, l3)]) #substitute in link lengths since they are constant
subs_dict = {t:0, theta_1:theta1_initial, theta_2:theta2_initial, theta_3:theta3_initial}
thetadot_c_initial = thetadot_c.subs(subs_dict)
thetadot1_initial = thetadot_c_initial[0]
thetadot2_initial = thetadot_c_initial[1]
thetadot3_initial = thetadot_c_initial[2]

#Separate out individual equations for angular speeds of link 1, 2, and 3
thetadot1_eq = thetadot_c[0]
thetadot2_eq = thetadot_c[1]
thetadot3_eq = thetadot_c[2]

#Initialize arrays for plotting
arraysize = int(duration/increment)
time = np.arange(0, duration, increment) #time array
theta1_array = [theta1_initial]*arraysize #link 1 angle array
theta2_array = [theta2_initial]*arraysize #link 2 angle array
theta3_array = [theta3_initial]*arraysize #link 3 angle array
thetadot1_array = [thetadot1_initial]*arraysize #link 1 angle vecloity array
thetadot2_array = [thetadot2_initial]*arraysize #link 2 angle velocity array
thetadot3_array = [thetadot3_initial]*arraysize #link 3 angle velocity array
x_array = [x_initial]*arraysize #end effector x position array
y_array = [y_initial]*arraysize #end effector y position array
xdot_array = [xdot_initial]*arraysize #end effector x speed array
ydot_array = [ydot_initial]*arraysize #end effector y speed array

for i in range(1, arraysize):
    #Calculate theta 1, 2, and 3 position from previous thetadot value
    theta1_array[i] = theta1_array[i-1]+(thetadot1_array[i-1]*increment) 
    theta2_array[i] = theta2_array[i-1]+(thetadot2_array[i-1]*increment)
    theta3_array[i] = theta3_array[i-1]+(thetadot3_array[i-1]*increment)
    
    #Calculate thetadot from current time and theta values
    thetadot1_array[i] = (thetadot1_eq.subs([(t, time[i]), (theta_1, theta1_array[i]), (theta_2, theta2_array[i]), (theta_3, theta3_array[i])])).evalf(5)
    thetadot2_array[i] = (thetadot2_eq.subs([(t, time[i]), (theta_1, theta1_array[i]), (theta_2, theta2_array[i]), (theta_3, theta3_array[i])])).evalf(5)
    thetadot3_array[i] = (thetadot3_eq.subs([(t, time[i]), (theta_1, theta1_array[i]), (theta_2, theta2_array[i]), (theta_3, theta3_array[i])])).evalf(5)
    

#Initialize arrays for link positions 
link1_x = [0]*arraysize
link1_y = [0]*arraysize
link2_x = [0]*arraysize
link2_y = [0]*arraysize
link3_x = [0]*arraysize
link3_y = [0]*arraysize

#Calculate link positions based on theta values to confirm results make sense
for i in range(arraysize):
    link1_x[i] = 15*sp.cos(theta1_array[i])
    link1_y[i] = 15*sp.sin(theta1_array[i])
    link2_x[i] = link1_x[i] + (15*sp.cos(theta1_array[i]+theta2_array[i]))
    link2_y[i] = link1_y[i] + (15*sp.sin(theta1_array[i]+theta2_array[i]))
    link3_x[i] = link2_x[i] + (2*sp.cos(theta1_array[i]+theta2_array[i]+theta3_array[i]))
    link3_y[i] = link2_y[i] + (2*sp.sin(theta1_array[i]+theta2_array[i]+theta3_array[i]))
        
# Create animation of links showing circular path 
i = 0
while i < arraysize:
    plt.clf()
    plt.plot(link3_x, link3_y)
    plt.plot([0, link1_x[i]],[0, link1_y[i]])
    plt.plot([link1_x[i], link2_x[i]],[link1_y[i], link2_y[i]])
    plt.plot([link2_x[i],link3_x[i]],[link2_y[i], link3_y[i]])
    plt.axis('square')
    plt.grid(True)
    plt.xlim([-30,30])
    plt.ylim([-30,30])
    plt.gca().set_aspect('equal', adjustable='box')
    i = i+5
    plt.pause(0.05)   
plt.show()

#Plot link angles and angular speeds
figure, axis = plt.subplots(2,1)
axis[0].plot(time, theta1_array, label='Theta1')
axis[0].plot(time, theta2_array, label='Theta2')
axis[0].plot(time, theta3_array, label='Theta3')
axis[0].set_title("link angles")
axis[0].legend()
axis[0].set_xlabel("Time [s]")
axis[0].set_ylabel("Angle [rad]")

axis[1].plot(time, thetadot1_array, label='Thetadot1')
axis[1].plot(time, thetadot2_array, label='Thetadot2')
axis[1].plot(time, thetadot3_array, label='Thetadot3')
axis[1].set_title("angle speeds")
axis[1].legend()
axis[1].set_xlabel('Time [s]')
axis[1].set_ylabel('Angular speed [rad/s]')
plt.tight_layout()
plt.show()




