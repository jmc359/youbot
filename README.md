# Youbot
**Repository for CPSC 472/572 Robot Design Project**
--------------

## TODO 
- **code documentation**: for each function, write a sentence description, the parameters, and what the function returns
- basic zombie avoidance behavior
- image processing
- wall detection and avoidance
- global berry map
- avoiding zombies based on color
- using the gripper

# Zombie Survival
Design of a controller for a robot to survive a harsh, challenging simulated world for as long as possible. For more information, see [world specifications](https://drive.google.com/file/d/1__hUKDCFwzRBgdBKmZmqddVbdgGxMGK4/view?usp=sharing).

## Turning Mechanisms

## Determining Safety

## Image Processing

## Helper Functions
- `getIndexOfMin()` 
    - returns the index of the minimum element in a given `float` array of `size_t` size
    - useful, alongside return values of safety functions (e.g., `calcZombiness()` and `isStuck()`, for determining the safest pane in view.
- `get_bearing_in_degrees()` 
    - returns a `double` which is the robot's heading in [0, 360] degree range
    - potentially useful for interpreting compass sensor readings to determine direction according to the global world state
    - however, currently implemented as a test for the compass sensor because of revision in turning specification for world 3 (i.e., "the robot can now only turn 90 degrees to the left or right").
- `get_bearing_in_radians()`
    - returns a `double` which is the robot's heading in [-1.57, 1.57] radian range
    - usage is similar to that of `get_bearing_in_degrees()`
    - however, currently implemented as a test for the compass sensor because of world 3 revised turning specification, see `get_bearing_in_degrees()`
 

## Exceptions/Misc. 

## Lambda (5) Team
- Rebecca Ramnauth, [rebecca.ramnauth@yale.edu](mailto:rebecca.ramnauth@yale.edu)
- Sydney Thompson, [sydney.thompson@yale.edu](sydney.thompson@yale.edu)
- Joe Connolly, [joe.connolly@yale.edu](joe.connolly@yale.edu)