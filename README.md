# Bug fix

To run this version copy the src folder inside a ros workspace and execute catkin_make. 
Then go to the src/assignment1_EXP/launch folder and run roslaunch sim_aruco.launch

Notice: this version of the project has been developed starting from the already existing package already sent.
I made the following changes in order to achieve a more reliable behavior in simulation as some issues arose while launching several times the code on my distribution:

# 1) cb_marker

Inside the file controller.py the function cb_marker() was updated so that the middle point of the marker is computed (in a slightly more direct way) by taking into consideration the id of the "edge".
This is the main change that was made.
It happened in rare occasions that the robot perceived in the same image two edges belonging to two different markers and computed a middle point not belonging to either of them. 
In this way we are guaranteed that the "self.center_point" structure is the middle point of a marker.

# 2) forward_state

Inside the file controller.py the forward_state() function was implemented with an open loop control. In this implementation a simple closed loop control is performed.

# 3) Tuning of parameters 

EXTRA_RANGE was reduced to achieve higher precision.
In order to achieve a more stable behavior all the velocities (angular and linear) were reduced, so that the overall evolution of the simulation is slow.
By doing so linear controllers, in many occasions, are no more required releasing a bit of computational burden and most importantly there should be no case in which a marker is not detected.
Having reduced the linear velocity a more precise distance to the current marker may be achieved so that, in this case, the robot ends up in a more favorable position with respect to the next marker.

# Conclusions

By statistical analysis performed on my distribution I strongly believe this version is more reliable (in the simulation environment) than the previously sent one.
Of course this doesn't logically imply anything regarding the formal correctness of any of either version.
But since some issues briefly mentioned before may be faced with the real robot as well, I felt dutiful to apply these changes. 
