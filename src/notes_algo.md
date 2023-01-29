# Algorithm TODOs & notes

## No avoidance possible in certain situations
If a robot to avoid literally stands in front of the robot to command,
then the algorithm instantly crashes because it doesn't find any solutions.

A possibility would be to compute a third vector that goes backward to avoid
the enemy, but only if it is necessary.

## Robot movement is too slow (wait=True)
Tried to put `wait` parameter to False multiple times, but since the current
control over the robot is done with a non-tuned PID controller, precision is lost.
At least with the current params, it runs.

## Possible waypoint computation is performed without taking in account avoid points distances
Here's the problem in a very simplified way :

* Controlled Robot needs to avoid someone in front of him (dist ~= 0.41m)
* The best waypoint is properly computed
* During computation, the best waypoint is considered as bad because its calculated trajectory towards destination point creates a collision with another robot
* Best waypoint is ignored, another waypoint is computed, but less efficient

Comes from the fact that, to check whether a waypoint is valid, we compute the direct trajectory from 
the waypoint towards the destination.

I'm not too sure about the best solution to fix that. Perhaps we should take in account the fact
that the other robot (that we won't collide with technically, but the affine function says the contrary)
is farther from the waypoint.

TL;DR take in account distance from wp to robot, compare it with distance from src to wp  
maybe there's another possibility