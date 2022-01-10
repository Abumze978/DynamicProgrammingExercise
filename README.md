# DynamicProgrammingExercise

## Problem Statement

The goal of this programming exercise is to deliver a package with a drone as quickly as
possible. To achieve this, the drone must first fly to a pick-up station to collect a package and
then reach a delivery station to discharge it. Along the way, the drone must avoid hazards such
as trees or angry residents who try to shoot it down.

![alt text](https://github.com/Abumze978/DynamicProgrammingExercise/blob/main/Screenshot%20(23).png)


The drone operates in a world that is represented by a M x N matrix. At each time-step the drone could apply one of the allowable control inputs: 
hover (stay), move north, move south, move east or move west. An input is not allowed if it makes the drone crash against 
a tree or makes the drone go outside the bounds.
At this point, a gust of wind can randomly move the drone in one direction with a given probability. 
If the drone hasn't yet crashed, the drone is exposed to the anger of the residents who try to shoot it down, with a probability that is dependent on the relative distance between the drone and the resident.
Finally, if the drone overcomes even this hurdle and is at a pick-up station carrying no package,
it collects one. If the drone has not crashed by this point and is at a delivery station
carrying a package, the task terminates. Whenever the drone crashes, it is brought to the base station and starts the next time step there without a package. This procedure takes a total of N_c time steps. 

## Resolution and Implementation
