# DynamicProgrammingExercise

The goal of this programming exercise is to deliver a package with a drone as quickly as
possible. To achieve this, the drone must first fly to a pick-up station to collect a package and
then reach a delivery station to discharge it. Along the way, the drone must avoid hazards such
as trees or angry residents who try to shoot it down.

![alt text](https://github.com/Abumze978/DynamicProgrammingExercise/blob/main/Screenshot%20(23).png)


The drone operates in a world that is represented by a M x N matrix. At each time-step the drone could apply one of the allowable control inputs: 
hover (stay), move north, move south, move east or move west. An input is not allowed if it makes the drone crash against 
a tree or makes the drone go outside the bounds.
