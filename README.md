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

## Moving Mechanisms
Functions and definitions for moving the robot base
- `translate()`
- `main()` --> `robot_control()`, describe parameters
- `robot_control()` --> `if(timer % 16 == 0)`

## Turning Mechanisms
Functions and definitions for turning the robot base
- `rotate()`
- `rotate_update()`
- `robot_control()` --> performing turn
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
- `rgb_to_hsv()`
- `in_range()`
- `color_mask_count()`

### Whole-Image Processing
- `get_views_vertical()`

## Determining Safety
Functions and definitions for evaluating the safety of the local environment
- `grayDeviation()`
- `calcZombiness()`
- `isStuck()`
- `robot_control()` --> computing safest route
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

## Lambda (5) Team
- Rebecca Ramnauth, [rebecca.ramnauth@yale.edu](mailto:rebecca.ramnauth@yale.edu)
- Sydney Thompson, [sydney.thompson@yale.edu](sydney.thompson@yale.edu)
- Joe Connolly, [joe.connolly@yale.edu](joe.connolly@yale.edu)