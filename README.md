# Differential Drive Robot

Obstacle avoidance DIRCOL trajectory optimization using [IPOPT.jl](https://github.com/jump-dev/Ipopt.jl) with [JuMP.jl](https://github.com/jump-dev/JuMP.jl).

Using [TrajOptPlots.jl](https://github.com/RoboticExplorationLab/TrajOptPlots.jl) for trajectory visualization.

## Simple trajectroy with obstacle avoidance
DDR starts at the orgin and drives to its goal pose while avoiding the red obstacle.
![](simple-traj-obs-avoidance.gif)
