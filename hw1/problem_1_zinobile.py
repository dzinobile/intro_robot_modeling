
import matplotlib.pyplot as plt
import numpy as np
import math

#initialize input variables
r = 0.3 #-------------wheel radius [m]
L = 1.2 #-------------distance between wheels [m]
wl = 2*math.pi #------left wheel speed [rad/s]
wr = math.pi #--------right wheel speed [rad/s]
x_initial = 0 #-------initial x position [m]
y_initial = 0 #-------initial y position [m]
theta_initial = 0 #---initial angle [rad]
T = 5 #---------------duration [s]
increment = 0.01 #----timestep [s]

#calculate theta dot 
theta_dot = (r*(wr-wl))/L #[rad/s]

#initialize arrays
arraysize = int(T/increment)
time = np.arange(0, T, increment)
theta = [0]*arraysize
x = [x_initial]*arraysize
y = [y_initial]*arraysize
x_dot = [0]*arraysize
y_dot = [0]*arraysize

#Fill theta, xdot, and ydot arrays
for i in range(arraysize):
    theta[i] = ((theta_dot*time[i])+theta_initial) #[rad]
    x_dot[i] = ((r*(wl+wr))/2)*math.cos(theta[i]) #[m/s]
    y_dot[i] = ((r*(wl+wr))/2)*math.sin(theta[i]) #[m/s]

#Fill x and y arrays
for i in range(1, arraysize):
    x[i] = x[i-1] + (x_dot[i]*increment) #[m]
    y[i] = y[i-1] + (y_dot[i]*increment) #[m]

#Plot path
plt.plot(x, y)
plt.axis('square')
plt.xlabel('x position [m]')
plt.ylabel('y position [m]')
plt.text(0,0, 'Start point')
plt.text(x[-1], y[-1], 'End point')
plt.show()
