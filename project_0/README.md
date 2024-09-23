This python script prompts the user for a travel time T, travel distance D, and choose scenario 1 or 2.

Scenario 1 moves the turtlebot the desired distance in the desired time at a constant velocity. 

Scenario 2 moves the turtlebot the desired distance in the desired time by accelerating for T/4 seconds, moving at a constant velocity for the next T/2 seconds, then decelerating for the final T/4 seconds. Acceleration and max velocity are calculated from these constraints. 
