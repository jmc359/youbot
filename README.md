# Youbot Zombie Survival
**Repository for CPSC 472/572 Robot Design Project**.
Design of a controller for a robot to survive a harsh, challenging simulated world for as long as possible. For more information, see [world specifications](https://drive.google.com/file/d/1__hUKDCFwzRBgdBKmZmqddVbdgGxMGK4/view?usp=sharing).

**Contents**
1. [Moving Mechanisms](https://github.com/jmc359/youbot#moving-mechanisms)
2. [Turning Mechanisms](https://github.com/jmc359/youbot#turning-mechanisms)
3. [Image Processing](https://github.com/jmc359/youbot#image-processing)
   - [Color Processing](https://github.com/jmc359/youbot#color-processing)
   - [Whole-Image Processing](https://github.com/jmc359/youbot#whole-image-processing)
4. [Evaluating Safety](https://github.com/jmc359/youbot#evaluating-safety)
5. [Helper Functions](https://github.com/jmc359/youbot#helper-functions)
6. [Further Steps](https://github.com/jmc359/youbot#further-steps)
7. [Team Members & Details](https://github.com/jmc359/youbot#lambda-5-team)

## Moving Mechanisms
Functions and definitions for moving the robot base
- `translate()` - Joe?
- `main()` &rarr; `robot_control()`, parameters
    - `timer`, a variable local to `main()` that is defined outside of `robot_control` so that it does not reset every time step
    - `turning` - Joe?
    - `timesteps` - Joe?
    - `threshold`, a safety threshold for initiating turn; because turning is time-consuming, it should be used sparsely and decivisely
    - `last_info` - Joe?
    - `robot_info` - Joe?
- `robot_control()` &rarr; `if(timer % 16 == 0)`, initiates camera captures and processing every other time step.

## Turning Mechanisms
Functions and definitions for turning the robot base
- `rotate()` - Joe?
- `rotate_update()` - Joe?
- `robot_control()` &rarr; performing turn
```c
// ... 
    if (zombieness > threshold) { // does calculated safety level warrant a turn?
          if (safest_direction == 0){ // turn left
            printf("ROTATING LEFT\n");
            rotate(LEFT, turning, timesteps);
          }
          else if (safest_direction == 1){ // turn right
            printf("ROTATING RIGHT\n");
            rotate(RIGHT, turning, timesteps);
          }
        }
    }
translate(FORWARD, turning); // after turn, continue moving forward
rotate_update(turning, timesteps);
```

## Image Processing
Functions and definitions for interpreting raw images taken of the local environment

### Color Processing
- `grayDeviation()`
    - returns how unlike gray a given rgb value is, the standard deviation of the `r`, `g`, and `b` values
    - can be implemented as a detector of 'stuck' situations. For example, the edge of the world is gray, a tree trunk/stump is black, and a wall can be any shade of gray depending on light source angling
- `rgb_to_hsv()` - Sydney?
- `in_range()` - Sydney?
- `color_mask_count()` - Sydney?

### Whole-Image Processing
- `get_views_vertical()`
    - splits a given camera image into a `viewpanes_horizontal` by `viewpanes_vertical` grid
    - currently implemented to split the image into vertical panes and evaluate safety per pane
    - improvements include: (i) calculating an incentive score, which considers berries according the robot's current `last.health` or `last.energy`, (ii) assessing horizontal panes for tree trunk/stump and berry detection, (iii) optimizing color classification methods such as using grayscaling or other color spaces

## Evaluating Safety
Functions and definitions for evaluating the safety of the local environment
- `calcZombiness()`
   - returns a calculation of how much 'zombieness' based on given frame's green and blue values
   - `pow(((g + b)/(255*2.0)), 10) * 100` is the sum of the green and blue values divided by the maximum green + blue value (255 * 2), then the result is magnified by a power of 10
   - an improved 'zombieness' calculator would classify a zombie by the peak in exactly two `r`, `g`, or `b` values
   - an even improved calculator would classify the zombie by its true color and movement/turning mechanisms could be alternate per case
- `isStuck()`
   - returns a `bool` of whether the robot is approaching a potential 'stuck' situation (i.e., wall, tree, world edge)
   - can be a factor for safety evaluation of current frame that can be prioritized alongside zombie detection
- `robot_control()` &rarr; computing safest route
```c
// split view into vertical panes and evaluate safety for each pane
float *views_vertical = get_views_vertical(image,viewpanes_vertical,viewpanes_horizontal,image_width,image_height);

// determine safest pane
int safest_pane = getIndexOfMin(views_vertical, (sizeof(views_vertical)/sizeof(views_vertical[0])));

// convert safest pane result to direction instruction (0 = 'left', 1 = 'right', 0.5 = 'straight')
double safest_direction = round(((float)safest_pane/(viewpanes_vertical - 1)) * 2)/2;

// determine total zombieness of safest pane
// compare to a threshold to determine if a turn is warranted
float zombieness = sumOfArray(&views_vertical[safest_pane], (sizeof(views_vertical)/sizeof(views_vertical[0])));

printf("safest pane = %d, safest direction = %.1f, zombieness = %.4f\n", safest_pane, safest_direction, zombieness);
```

## Helper Functions
Helper methods for tests and aforementioned processes
- `getIndexOfMin()` 
    - returns the index of the minimum element in a given `float` array of `size_t` size
    - useful, alongside return values of safety functions (e.g., `calcZombiness()` and `isStuck()`, for determining the safest pane in view.
- `sumOfArray()`
    - returns a `float` sum of all elements in a `float` array of size `n`
    - useful for calculating safety, a component of which is the total zombieness in a given frame
- `get_bearing_in_degrees()` 
    - returns a `double` which is the robot's heading in [0, 360] degree range
    - potentially useful for interpreting compass sensor readings to determine direction according to the global world state
    - however, currently implemented as a test for the compass sensor because of revision in turning specification for world 3 (i.e., "the robot can now only turn 90 degrees to the left or right").
- `get_bearing_in_radians()`
    - returns a `double` which is the robot's heading in [-1.57, 1.57] radian range
    - usage is similar to that of `get_bearing_in_degrees()`
    - however, currently implemented as a test for the compass sensor because of world 3 revised turning specification, see `get_bearing_in_degrees()`
 
## Further Steps
1. **improving default zombie avoidance behavior**
    - adjusting safety threshold to 'learn' from environment
    - separating zombie avoidance from general safety evaluations
    - creating a priority list for handling zombie avoidance situations, such as choosing between purple versus aqua zombie, staying close to wall or berry location according to the circumstance
2. **optimizing current image processing**
    - Sydney?
3. **improving wall detection and avoidance**
4. **optimizing berry detection and collection**
    - creating a global berry map that allows the robot to 'remember' general location of berries
    - implement a 'berry necessity' function that evaluates when to ignore a found berry or initiate a berry search
    - implement 'learning' for berry effects; for example, avoid yellow berries if they often decrease energy
5. **avoiding zombies based on color**
6. **using the gripper**, exploring the use of the gripper for getting berries on top tree stumps

## Lambda (5) Team
- Rebecca Ramnauth, [rebecca.ramnauth@yale.edu](mailto:rebecca.ramnauth@yale.edu)
- Sydney Thompson, [sydney.thompson@yale.edu](sydney.thompson@yale.edu)
- Joe Connolly, [joe.connolly@yale.edu](joe.connolly@yale.edu)