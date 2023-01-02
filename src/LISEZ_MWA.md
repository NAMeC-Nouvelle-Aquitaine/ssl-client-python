# About the program
## Initial setup
Put the robot on the circle that is around the center of the field.  
Anywhere on the circle will do, but NOT at its center !

I had no idea if distances were going to be in millimeters or meters, so there's a nasty hack in there.

If you don't put the robot on the circle around the center, the hack won't work.
Worst thing happening is the robot will almost not move, so that's not a big deal.

## What does the program do
Two main tests are in the program.
Before any test is launched, the robot will go to the center of the field at a unknown speed, so watch out for that

### Slow goto implemented
Everytime the robot moves, it will do two things :
> 
>* Try to slowly attain the destination point
>* Process a go to that will correct the position to the wanted destination

So once you think a robot is not moving anymore, don't trust it.  
Wait for the prompt `Attained (x, y)`  
This is when it'll have really finished
### Rotating test
Makes the robot rotate, left and right.  

That's it   
What else do you want ?
### Movement test
Robot goes to center, then runs around all the field by starting from the top left corner (at negative x and y coordinates).
5 waypoints are set, and each time the robot will point towards this waypoint
* Top mid
* Top left corner
* Bottom left corner
* Bottom right corner
* Top right corner

## Launching
Just launch `main.py` and the robot will be going around the field at a very slow pace.  
Recall that values are in millimeters in the program  
I advise not changing the speed