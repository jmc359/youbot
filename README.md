# Youbot Zombie Survival
**Repository for CPSC 472/572 Robot Design Project**.
Design of a controller for a robot to survive a harsh, challenging simulated world for as long as possible. For more information, see [world specifications](https://drive.google.com/file/d/1__hUKDCFwzRBgdBKmZmqddVbdgGxMGK4/view?usp=sharing).

**Contents**
1. [Robot Control](https://github.com/jmc359/youbot#robot-control)
2. [Moving Mechanisms](https://github.com/jmc359/youbot#moving-mechanisms)
3. [Image Processing](https://github.com/jmc359/youbot#image-processing)
   - [Color Processing](https://github.com/jmc359/youbot#color-processing)
   - [Whole-Image Processing](https://github.com/jmc359/youbot#whole-image-processing)
4. [Evaluating Safety](https://github.com/jmc359/youbot#evaluating-safety)
5. [Helper Functions](https://github.com/jmc359/youbot#helper-functions)
6. [Further Steps](https://github.com/jmc359/youbot#further-steps)
7. [Team Members & Details](https://github.com/jmc359/youbot#lambda-5-team)

## Robot Control
Functions and definitions for overall robot control

- `main()` &rarr; `robot_control()`, 
    - control parameters
      - `timer`
        - a variable local to `main()` that is defined outside of `robot_control` so that it does not reset every time step
      - `last_gps`
        - last value of the gps, stored for comparison in determining whether the robot has gotten itself stuck
      - `turning`
        - boolean flag indicating if robot is turning
      - `timesteps`
        - keeps track of how many timesteps the robot has spent turning
      - `stuck_steps` 
        - keeps track of how long the robot has been stuck while trying to move forward
      - `time` 
        - keeps track of time spent in total timesteps
      - `zombie_threshold` 
        - a safety threshold for initiating turn loosely corresponding to how close the zombie is in view of a camera
      - `berry_threshold`
        - a threshold for determining when to navigate toward berries
      - `obstacle_threshold` 
        - a threshold for determining when to engage in obstacle avoidance
      - `last_info` 
        - last robot control info (health, energy, armor) used for comparison and behavior selection
      - `robot_info` 
        - current robot control info (health, energy, armor) used for comparison and behavior selection

    - `robot_control()` 
      - initiates camera captures and processing every other time step
      - arbitrates subsumptive behavior based on sensor inputs and control parameters above

## Moving Mechanisms
Functions and definitions for moving/turning the robot base
- `translate()` 
  - Moves the robot forward or backward based on a direction parameter
- `rotate()` 
  - rotates the robot according to the direction parameters
- `rotate_update()` 
  - updates the turning timer (timesteps) and waits until that timer reaches a threshold before enabling translation again
- `robot_control()` &rarr; performing turn
```c
  {
    // ... 
    int direction = analyze_cameras(...);

    // ...

    // make_turn
    if (direction == RIGHT || direction == LEFT) {
        rotate(direction, &(params->turning), &(params->timesteps), params->q, params->time);
    }
  }
  translate(FORWARD, &(params->turning));
  rotate_update(&(params->turning), &(params->timesteps));

```

## Image Processing
Functions and definitions for interpreting raw images taken of the local environment

### Color Processing
- `grayDeviation()`
    - returns how unlike gray a given rgb value is, the standard deviation of the `r`, `g`, and `b` values
    - can be implemented as a detector of 'stuck' situations. For example, the edge of the world is gray, a tree trunk/stump is black, and a wall can be any shade of gray depending on light source angling
- `rgb_to_hsv()`
   - converts RGB color scheme to HSV for more accurate color comparison across shadows
   - modifies a pointer to hsv[3] array with the corresponding hue, saturation, and value components
- `in_range()`
   - determines whether a color is within `hue_epsilon` of target hue and `epsilon` of target saturation and value for a given color
   - built to be a helper function for `color_mask_image()`, as it is used to compare each pixel value to target colors for zombies, berries, walls, obstacles, etc.
- `color_mask_image()`
   - creates a binary mask for a particular image of size `image_height` x `image_width` by comparing image pixel values with given RGB comparison color
   - evaluated pixel-by-pixel, where each pixel in the mask has the boolean value `in_range(hsv_pixel, target_hsv_color)`.

### Whole-Image Processing
- `get_views_vertical_mask()`
    - splits a color mask into a `viewpanes_horizontal` by `viewpanes_vertical` grid
    - currently implemented to split the mask into vertical panes and evaluate safety per pane
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
- `compute_color_viewpane_sums()`
   - computes the percentage of panel of a particular object type - zombie, berry, or obstacle - based on an input color collection
   - a helper function for `analyze_camera_view()` to decrease code duplication
- `analyze_camera_view()`
   - computes image masks for zombies, berries, and obstacles for a particular camera (front, left, or right)
   - modifies 1-D arrays of length `viewpanes_vertical` to track percentage of panel that is 'berry', 'zombie', or 'obstacle'
- `analyze_cameras()`
   - for the three cameras used - front, left, and right - synthesize the masked images from all three cameras to determine global behavior
   - follows a behavior heirarchy as follows:
      - if a collision is imminent, turn away
      - otherwise, avoid any visible zombies if they are too close
      - in safe situations (no impending collision or zombies) move towards visible berries
      - if no berries visible, drive forward
   - the activation of each behavior is thresholded by hyperparameters of the system
- `override()`
  - makes sure that the robot does not get caught in turn cycles by investigating the previous few turns in the history and comparing turn times and directions
- `robot_control()` &rarr; controls distribution of direction controls to robot
   - given a control from `analyze_cameras(...)`, computes whether to send controls as prescribed or override behavior with more urgent controls
   - overrides direction from `analyze_cameras()` if the robot is stuck in a loop or an edge/corner
```c
{
    // ... 
    int direction = analyze_cameras(...);

    // override if turning in cycles
    int override_direction = override(params->q);

    // Turn if stuck
    if (override_direction >= 0){
      printf("OVERRIDING BAD BEHAVIOR\n");
      direction = override_direction;
    }
    // if losing health from zombie, run forward
    else if (params->last_info.health - params->current_info.health > 3) {
        printf("OVERRIDING TO RUN\n");
        direction = FORWARD;
    }
    // Get unstuck from walls/trees/edges/etc.
    else if (is_stuck(params->last_gps, gps, &(params->stuck_steps), &(params->turning), &(params->timesteps))){
      printf("GETTING UNSTUCK\n");
      direction = LEFT;
    }

    // make_turn
    if (direction == RIGHT || direction == LEFT) {
        rotate(direction, &(params->turning), &(params->timesteps), params->q, params->time);
    }
}
translate(FORWARD, &(params->turning));
rotate_update(&(params->turning), &(params->timesteps));
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
1. **optimizing berry detection and collection**
    - creating a global berry map that allows the robot to 'remember' general location of berries
    - implement a 'berry necessity' function that evaluates when to ignore a found berry or initiate a berry search
    - implement 'learning' for berry effects; for example, avoid yellow berries if they often decrease energy
2. **avoiding zombies based on color**
3. **using the gripper**, exploring the use of the gripper for getting berries on top tree stumps

## Lambda (5) Team
- Rebecca Ramnauth, [rebecca.ramnauth@yale.edu](mailto:rebecca.ramnauth@yale.edu)
- Sydney Thompson, [sydney.thompson@yale.edu](sydney.thompson@yale.edu)
- Joe Connolly, [joe.connolly@yale.edu](joe.connolly@yale.edu)
