/*
 * Copyright 1996-2019 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Starts with a predefined behaviors and then
 *                read the user keyboard inputs to actuate the
 *                robot
 */

#include <webots/keyboard.h>            
#include <webots/robot.h>
#include <webots/supervisor.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/camera.h>
#include <webots/accelerometer.h>
#include <webots/lidar.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/range_finder.h>
#include <webots/gyro.h>
#include <webots/light_sensor.h>
#include <webots/receiver.h>
#include <webots/distance_sensor.h>

#include <youbot_zombie_1.h>

//void wb_camera_enable(WbDeviceTag tag, int sampling_period);
//void wb_camera_disable(WbDeviceTag tag);
//int wb_camera_get_sampling_period(WbDeviceTag tag);

int robot_angle = 0;
#define TIME_STEP 32

//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// ONLY USE THE FOLLOWING FUNCTIONS TO MOVE THE ROBOT /////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////


void stop()
{
  base_reset();
}

void go_forward()
{
  base_forwards();
}

void go_backward()
{
  base_backwards();
}

void turn_left()
{
  base_turn_left();
  robot_angle = robot_angle + 90;
  if (robot_angle == 360)
    robot_angle = 0;

}

void turn_right()
{
  base_turn_right();  
  robot_angle = robot_angle - 90;
  if (robot_angle == -90)
    robot_angle = 270;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// CHANGE CODE BELOW HERE ONLY ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 * Queue functions
 */

// Link in queue
typedef struct turn{
  int direction;
  long long steps;
  struct turn *next;
} Turn;

// Queue data structure
typedef struct queue{
  struct turn* head;
  struct turn* tail;
  int n;
} Queue;

// construct queue
Queue *queue_constuct(void){
    Queue *q;
    q = malloc(sizeof(Queue));
    q->head = 0;
    q->tail = 0;
    q->n = 0;
    return q;
}

// return queue length
int queue_length(Queue *q){
  return q->n;
}

// dequeue element from q
Turn *dequeue(Queue *q){
  if (q->n > 0){
    Turn *t = q->head;
    q->head = t->next;
    q->n--;
    return t;
  }
  else{
    printf("ERROR: Tried to dequeue from empty queue!");
    return 0;
  }
}

// enqueue struct turn to q
void enqueue(Queue *q, int direction, long long steps){
  Turn *t = malloc(sizeof(Turn));
  t->next = 0;
  t->direction = direction;
  t->steps = steps;

  if (q->n == 0){
    q->head = q->tail = t;
  }
  else {
    q->tail->next = t;
    q->tail = t;
  }
  q->n++;
  if (q->n > 4){
    Turn *tmp = dequeue(q);
    free(tmp);
  }
}

// print elements of queue
void print_queue(Queue *q){
  Turn *t = q->head;
  while (t != NULL){
    printf("TURN: %d TIME: %lld\n", t->direction, t->steps);
    t = t->next;
  }
}

// destroy queue
void queue_destroy(Queue *q){
  while (queue_length(q) > 0){
    Turn *tmp = dequeue(q);
    free(tmp);
  }
}

// override bad turning behavior
int override(Queue *q){
  if (q->n > 1){
    Turn *t = q->head;
    int override = 1;
    for (int i = 0; i < q->n-1; i++){
      if (t->next->steps - t->steps > 300 || t->next->direction == t->direction){
        override = 0;
      }
      t = t->next;
    }
    if (override){
      return t->direction;
    }
  }
  return -1;
}


/*
 * Helper functions for printing/creating/returning min of arrays
 */

// returns index of minimum int in @array
int getIndexOfMin(float* array, size_t size){
  int minimum = 0;

  for (int i = 1; i < size; i++)
  {
      if (array[i] < array[minimum])
         minimum = i;
  }
  return minimum;
}

void print_array(float array[], int size) {
    for (int i = 0; i < size; i++) {
        printf("%f ", array[i]);
    }
    printf("\n");
}

// calculate sum of all elements in given float array
float sumOfArray(float a[], int n) {
  float sum = 0;

  for(int i = 0; i < n; i++)
    sum += a[i];

  return sum;
}

/*
 * Functions for getting bearing of robot
 */

// returns heading in [0, 360] degree range
double get_bearing_in_degrees() {
  const double *north = wb_compass_get_values(3);
  double rad = atan2(north[0], north[2]);
  double bearing = (rad - 1.5708) / M_PI * 180.0;
  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}

// returns heading in [-1.57, 1.57] radian range
double get_bearing_in_radians() {
  const double *north = wb_compass_get_values(3);
  return atan2(north[0], north[2]);
}

/*
 * Functions and definitions for moving the robot base
 */

// defined directions for rotating/translating
#define LEFT (0)
#define RIGHT (1)
#define FORWARD (2)
#define BACKWARD (3)

// Initiates turning the robot
// rotate_update() waits the proper amount of time
// after this function is called
void rotate(int direction, int *turning, int *timesteps, Queue *q, long long time){
  if ((*timesteps) == 0){ // Initiate turn at first timestep
    stop();
    (*turning) = 1;
    enqueue(q, direction, time);
    switch(direction){
      case LEFT:  turn_right(); printf("TURNING LEFT\n"); break; // **original functions are swapped**
      case RIGHT: turn_left(); printf("TURNING RIGHT\n"); break;
    }
  }
}

// Updates timestep counter for turning after turn is initiated
// Updates counter if and only if the robot is in the process of turning
void rotate_update(int *turning, int *timesteps){
  if ((*turning)){
    if ((*timesteps) < 150){ // only count when turning
      (*timesteps)++;
    }
    else{ // reset after waiting 150 steps
      (*turning) = 0;
      (*timesteps) = 0;
    }
  }
}

// Translates forward/backward only if not turning
void translate(int direction, int *turning){
  if(!(*turning)){
    switch(direction){ // Only translate when not turning
      case FORWARD:  go_forward(); break;
      case BACKWARD: go_backward(); break;
    }
  }
}

// handles getting stuck near edge of world/wall/etc.
int is_stuck(double *last_gps, const double *gps, int *steps, int *turning, int *timesteps){
  double diff0 = fabs(gps[0] - last_gps[0]);
  double diff2 = fabs(gps[2] - last_gps[2]);
  // printf("Delta: %f %f %f\n", diff0, diff1, diff2);
  if(!(*turning)){
    if (diff0 < 0.002 && diff2 < 0.002){
      if (*steps <= 1){
        (*steps)++;
      }
      else{
        return 1;
      }
    }
    else{
      (*steps) = 0;
    }
  }
  else{
    (*steps) = 0;
  }
  return 0;
}


/*
 * Functions for safety/zombie logic
 */

// logic for determining how much zombie in frame
// could also be used for determining 'usefulness'
// considering berries
float calcZombiness(int g, int b){
  return pow(((g + b)/(255*2.0)), 10) * 100;
}


/*
 * Functions for image processing (masking/zombiness/etc.)
 */

#define BLUE 0
#define AQUA 1
#define GREEN 2
#define PURPLE 3
#define RED 4
#define PINK 5
#define ORANGE 6
#define YELLOW 7
#define WHITE 8
#define BLACK 9

static int zombie_colors[4] = {BLUE, AQUA, GREEN, PURPLE};
static int berry_colors[4] = {RED, PINK, ORANGE, YELLOW};
static int wall_colors[1] = {WHITE};
static int obstacle_colors[1] = {BLACK};

static int min_colors[10][3] = {{10, 39, 97}, {11, 67, 68}, {11, 51, 16}, {45, 18, 92},
                               {69, 18, 17}, {62, 38, 67}, {29, 175, 31}, {69, 67, 13},
                               {69, 74, 94}, {10, 10, 13}};
static int max_colors[10][3] = {{26, 96, 180}, {21, 152, 129}, {37, 192, 41}, {115, 49, 185},
                               {209, 62, 44}, {193, 124, 167}, {194, 124, 85}, {207, 195, 37},
                               {222, 223, 239}, {35, 36, 38}};

void rgb_to_hsv(int rgb[], double *hsv) {
    double red = rgb[0]/255.0;
    double green = rgb[1]/255.0;
    double blue = rgb[2]/255.0;

    double c_max = 0;
    double c_min = 0;
    double delta = c_max - c_min;
    double hue, saturation, value;

    if (red >= blue && red >= green) {
        c_max = red;
    } else if (green >= red && green >= blue) {
        c_max = green;
    } else if (blue >= red && blue >= green) {
        c_max = blue;
    }

    if (red <= blue && red <= green) {
        c_min = red;
    } else if (green <= red && green <= blue) {
        c_min = green;
    } else if (blue <= red && blue <= green) {
        c_min = blue;
    }

    delta = c_max - c_min;

    if (c_max == red) {
        hue = (green - blue)/delta;
    } else if (c_max == green) {
        hue = 2 + (blue - red) / delta;
    } else if (c_max == blue) {
        hue = 4 + (red - green) / delta;
    } else {
        hue = 0;
    }

    hue *= 60;
    if (hue < 0) {
        hue += 360;
    }

    if (c_max == 0) {
        saturation = 0;
    } else {
        saturation = 100 * delta / c_max;
    }

    value = 100 * c_max;

    hsv[0] = hue;
    hsv[1] = saturation;
    hsv[2] = value;
}

// function computing whether given hsv value is close to given value
int in_range(double *hsv, double *hsv_compare, double hue_epsilon, double epsilon)
{
    int h_valid = abs((int)hsv_compare[0] - (int)hsv[0])%360 < hue_epsilon;
    int s_valid = fabs(hsv_compare[1] - hsv[1]) < epsilon;
    int v_valid = fabs(hsv_compare[2] - hsv[2]) < epsilon;

    int color_in_range = (h_valid) && (s_valid) && (v_valid);
    return color_in_range;
}

// create image masked on input color
void color_mask_image(const unsigned char *image, int color, int image_width, int image_height, int mask_image[image_width][image_height]) {
    double hue_epsilon = 5.0;
    double epsilon = 20.0;

    double hsv_min[3];
    rgb_to_hsv(min_colors[color], hsv_min);
    double hsv_max[3];
    rgb_to_hsv(max_colors[color], hsv_max);

//   FILE *file_pointer = NULL;
//   file_pointer = fopen("image_mask.txt", "w");
//
//   FILE *im_fptr_r = NULL;
//   im_fptr_r = fopen("image_r.txt", "w");
//
//   FILE *im_fptr_g = NULL;
//   im_fptr_g = fopen("image_g.txt", "w");
//
//   FILE *im_fptr_b = NULL;
//   im_fptr_b = fopen("image_b.txt", "w");

    // int color_count = 0;
    for (int x = 0; x < image_width; x++) {
        for (int y = 0; y < image_height; y++) {
            int r = wb_camera_image_get_red(image, image_width, x, y);
            int g = wb_camera_image_get_green(image, image_width, x, y);
            int b = wb_camera_image_get_blue(image, image_width, x, y);

            int rgb[3] = {r, g, b};
            double hsv[3];
            rgb_to_hsv(rgb, hsv);

            int in_range_min = in_range(hsv, hsv_min, hue_epsilon, epsilon);
            int in_range_max = in_range(hsv, hsv_max, hue_epsilon, epsilon);
            int found_color = in_range_min || in_range_max;
            mask_image[x][y] = found_color;

            // color_count += found_color;

            // if (x == 65 && y == 30) {
                // printf("xy found: {%d, %d, %d}, {%f, %f, %f}\n", r, g, b, hsv[0], hsv[1], hsv[2]);
                // printf("hsv min: {%f, %f, %f}\n", hsv_min[0], hsv_min[1], hsv_min[2]);
                // printf("hsv max: {%f, %f, %f}\n", hsv_max[0], hsv_max[1], hsv_max[2]);
            // }

//            if (y == 0) {
//                    fprintf(file_pointer, "%d", found_color);
//                    fprintf(im_fptr_r, "%d", r);
//                    fprintf(im_fptr_g, "%d", g);
//                    fprintf(im_fptr_b, "%d", b);
//                } else {
//                    fprintf(file_pointer, ",%d", found_color);
//                    fprintf(im_fptr_r, ",%d", r);
//                    fprintf(im_fptr_g, ",%d", g);
//                    fprintf(im_fptr_b, ",%d", b);
//            }
        }
//       fprintf(file_pointer, "\n");
//       fprintf(im_fptr_r, "\n");
//       fprintf(im_fptr_g, "\n");
//       fprintf(im_fptr_b, "\n");
    }
//   fclose(file_pointer);
//   fclose(im_fptr_r);
//   fclose(im_fptr_g);
//   fclose(im_fptr_b);
//    printf("Color count: %d\n", color_count);
}

// logic for determining how much zombie in frame
// could also be used for determining 'usefulness'
// considering berries

// calculate how unlike gray given rgb values are
double grayDeviation(int r, int g, int b){
  double red = r/255.0;
  double green = g/255.0;
  double blue = b/255.0;

  double avg = (red + green + blue)/3; // calculate mean
  double sum = // calculate sum of squared differences
    pow(avg - red, 2)
  + pow(avg - green, 2)
  + pow(avg - blue, 2);
  return sqrt(sum); // return standard deviation
}

bool isStuck(int r, int g, int b){
  // float score = 0;
  double stuckness = grayDeviation(r, g, b);
  if (stuckness < 0.05) { // currently an arbitrary threshold
    // potentially approaching a 'stuck' situation (e.g., wall, trunk, world edge)
    // score += 1;
    return true;
  }
  return false;

  /* // can extend to calculating safety
  double average = (r + g + b)/3;
  int zombie_values = 0;
  if (((average - r/255.0)/stuckness) > 1.5) zombie_values += 1;
  if (((average - g/255.0)/stuckness) > 1.5) zombie_values += 1;
  if (((average - b/255.0)/stuckness) > 1.5) zombie_values += 1;

  if (zombie_values == 2 || b > 50) { // if zombie is of exactly two colors or blue is present
    score += 1;
  }
  */
}

// return vertical panes from image
float *get_views_vertical_mask(int viewpanes_vertical, int viewpanes_horizontal, int image_width, int image_height, int mask_array[image_width][image_height], bool bottom_only){
  float viewpanes[viewpanes_vertical][viewpanes_horizontal];
  for (int vx = 0; vx < viewpanes_vertical; vx++){
    for (int vy = 0; vy < viewpanes_horizontal; vy++){
      // define start and end for current vertical pane
      int start_vx = (image_width/viewpanes_vertical) * vx;
      int end_vx = (image_width/viewpanes_vertical) * (vx + 1);

      // define start and end for current horizontal pane
      int start_vy = (image_height/viewpanes_horizontal) * vy;
      int end_vy = (image_height/viewpanes_horizontal) * (vy + 1);

      // get sum of active pixels in mask
      int mask_sum = 0;
      for (int x = start_vx; x < end_vx; x++){
        for (int y = start_vy; y < end_vy; y++){
          // compute rgb values of current pixel
            mask_sum += mask_array[x][y];
        }
      }

      // compute average rgb of current pane
      int total_pix = (end_vx - start_vx) * (end_vy - start_vy);
      float mask_avg = (float)mask_sum / total_pix;
//      printf("V_V=%d [start=%3d, end=%3d]; V_H=%d [start=%3d, end=%3d]; mask sum=%d; zombieness=%f\n", vx, start_vx, end_vx, vy, start_vy, end_vy, mask_sum, mask_avg);
      mask_sum = 0;

      // store 'zombieness' computation
      viewpanes[vx][vy] = mask_avg;
      // printf("V_V=%d [start=%3d, end=%3d]; V_H=%d [start=%3d, end=%3d]; zombieness=%f\n", vx, start_vx, end_vx, vy, start_vy, end_vy, viewpanes[vx][vy]);
    }
  }

  // get vertical panes only
  float *views_vertical = malloc(sizeof(float) * viewpanes_vertical);
  // float views_vertical[viewpanes_vertical];
  for (int row = 0; row < viewpanes_vertical; row++){
    float sum = 0;
    if (bottom_only) {
        for (int col = 1; col < viewpanes_horizontal; col++){
          sum += viewpanes[row][col];
        }
    } else {
        for (int col = 0; col < viewpanes_horizontal; col++){
          sum += viewpanes[row][col];
        }
    }
    views_vertical[row] = 100*sum;
  }
  return views_vertical;
}

// computes a sum over colors by viewpanes
// this function is for detecting/seeking berries and avoiding zombies and walls
void compute_color_viewpane_sums(int viewpanes_vertical, int viewpanes_horizontal, int image_width, int image_height, const unsigned char *image,
                                 int num_colors, int colors[num_colors], float color_sums[viewpanes_vertical], bool bottom_only)
{
    float factor = 1;
    for (int i = 0; i < viewpanes_vertical; i++) {
        color_sums[i] = 0;
    }

    for (int i = 0; i < num_colors; i++) {
        factor = 1;
        int color = colors[i];
        if (color == PURPLE) {
            factor = 2;
        }
        int mask_image[image_width][image_height];
        color_mask_image(image, color, image_width, image_height, mask_image);

        float *views_vertical = get_views_vertical_mask(viewpanes_vertical,viewpanes_horizontal,image_width,image_height, mask_image, bottom_only);
        for (int j = 0; j < viewpanes_vertical; j++) {
            color_sums[j] += factor * views_vertical[j];
        }
        free(views_vertical);
    }
}

/*
 * Main robot control function, called every time step
 */

typedef struct control{
  double *last_gps;
  int turning, timesteps, stuck_steps;
  float zombie_threshold, berry_threshold, obstacle_threshold;
  float zombie_sensitivity, berry_sensitivity, obstacle_sensitivity;
  Queue *q;
  struct Robot current_info, last_info;
  long long time;
} Control;

// perform computations to analyze danger/berriness/obstruction of camera view
void analyze_camera_view(int viewpanes_vertical, int viewpanes_horizontal, int image_width, int image_height, const unsigned char* image,
                         float total_danger[viewpanes_vertical], float total_berries[viewpanes_vertical], float total_obstacles[viewpanes_vertical])
{
    int num_zombie_colors = 4;
    compute_color_viewpane_sums(viewpanes_vertical, viewpanes_horizontal, image_width, image_height, image,
                                num_zombie_colors, zombie_colors, total_danger, 0);
    // printf("Total danger: ");
    // print_array(total_danger, viewpanes_vertical);

    // check for berries
    int num_berry_colors = 4;
    compute_color_viewpane_sums(viewpanes_vertical, viewpanes_horizontal, image_width, image_height, image,
                                num_berry_colors, berry_colors, total_berries, 1);
    // printf("Total berries: ");
    // print_array(total_berries, viewpanes_vertical);

    // check for obstacles
    int num_obstacle_colors = 1;
    compute_color_viewpane_sums(viewpanes_vertical, viewpanes_horizontal, image_width, image_height, image,
                                num_obstacle_colors, obstacle_colors, total_obstacles, 0);
    // printf("Total obstacles: ");
    // print_array(total_obstacles, viewpanes_vertical);
}

int analyze_cameras(Control *params, int viewpanes_vertical, int viewpanes_horizontal, int front_image_width, int front_image_height,
                    int right_image_width, int right_image_height, int left_image_width, int left_image_height) {
    const unsigned char *front_image = wb_camera_get_image(4);
    const unsigned char *right_image = wb_camera_get_image(9);
    const unsigned char *left_image = wb_camera_get_image(10);

    printf("Right camera:\n");
    float right_total_danger[viewpanes_vertical];
    float right_total_berries[viewpanes_vertical];
    float right_total_obstacles[viewpanes_vertical];
    analyze_camera_view(viewpanes_vertical, viewpanes_horizontal, right_image_width, right_image_height, right_image,
                        right_total_danger, right_total_berries, right_total_obstacles);

    printf("Left camera:\n");
    float left_total_danger[viewpanes_vertical];
    float left_total_berries[viewpanes_vertical];
    float left_total_obstacles[viewpanes_vertical];
    analyze_camera_view(viewpanes_vertical, viewpanes_horizontal, left_image_width, left_image_height, left_image,
                        left_total_danger, left_total_berries, left_total_obstacles);

    printf("Front camera:\n");
    float front_total_danger[viewpanes_vertical];
    float front_total_berries[viewpanes_vertical];
    float front_total_obstacles[viewpanes_vertical];
    analyze_camera_view(viewpanes_vertical, viewpanes_horizontal, front_image_width, front_image_height, front_image,
                        front_total_danger, front_total_berries, front_total_obstacles);

    int mask_image[front_image_width][front_image_height];
    color_mask_image(left_image, PINK, front_image_width, front_image_height, mask_image);

    // determine direction from camera inputs
    int direction = FORWARD; // move forward by default

    // check for immediate danger
    float front_danger_sum = 0;
    float right_danger_sum = 0;
    float left_danger_sum = 0;

    for (int i = 0; i < viewpanes_vertical; i++) {
        if (i > 0 && i < viewpanes_vertical-1) {
          front_danger_sum += front_total_danger[i];
          right_danger_sum += right_total_danger[i];
          left_danger_sum += left_total_danger[i];
        }
    }

    // check for immediate berries
    float front_berry_sum = sumOfArray(front_total_berries, viewpanes_vertical);
    // float right_berry_sum = sumOfArray(right_total_berries, viewpanes_vertical);
    // float left_berry_sum = sumOfArray(left_total_berries, viewpanes_vertical);

    // check for immediate obstacles
    float front_obstacle_sum = sumOfArray(front_total_obstacles, viewpanes_vertical);
    float right_obstacle_sum = sumOfArray(right_total_obstacles, viewpanes_vertical);
    float left_obstacle_sum = sumOfArray(left_total_obstacles, viewpanes_vertical);
    
    // calculate the number of berries on peripheral boxes
    // bool right_berries_on_boxes = right_obstacle_sum > 1.5*right_berry_sum;
    // bool left_berries_on_boxes = left_obstacle_sum > 1.5*left_berry_sum;

    // printf("right berries on boxes: %d\n", right_berries_on_boxes);
    // printf("left berries on boxes: %d\n", left_berries_on_boxes);

    int center_frame = viewpanes_vertical / 2;
    
    // initialize direction incentive
    float front_incentive = 0, left_incentive = 0, right_incentive = 0;
    
    // adjust sensitivity to berries according to current health
    float remaining_energy = (100-params->current_info.energy)/100.0;
    float berry_sensitivity = (remaining_energy > params->berry_threshold) ? remaining_energy : params->berry_sensitivity; 
    printf("remaining_energy = %.3f, berry_sensitivity = %.2f\n", remaining_energy, berry_sensitivity);
    
    // determine berry incentive 
    if (left_total_berries[center_frame] > right_total_berries[center_frame] && left_total_berries[center_frame] > front_berry_sum) {
        left_incentive += berry_sensitivity;
    } else if (right_total_berries[center_frame] > left_total_berries[center_frame] && right_total_berries[center_frame] > front_berry_sum) {
        right_incentive += berry_sensitivity;
    } else {
        front_incentive += berry_sensitivity;
    }

    // determine zombie avoidance incentive
    if (left_danger_sum < right_danger_sum && left_danger_sum < front_danger_sum) {
        left_incentive += params->zombie_sensitivity;
    } else if (right_danger_sum < left_danger_sum && right_danger_sum < front_danger_sum) {
        right_incentive += params->zombie_sensitivity;
    } else {
        front_incentive += params->zombie_sensitivity;
    }

    // determine obstacle avoidance incentive
    if (left_obstacle_sum > right_obstacle_sum) {
        right_incentive += params->obstacle_sensitivity;
    } else if (right_obstacle_sum > left_obstacle_sum){
        left_incentive += params->obstacle_sensitivity;
    } else {
        front_incentive += params->obstacle_sensitivity;
    }
     
     // choose a direction according to incentive consensus
     printf("incentives [left, front, right] = [%.2f, %.2f, %.2f]\n", left_incentive, front_incentive, right_incentive);
     if (front_incentive >= left_incentive && front_incentive >= right_incentive) direction = FORWARD;
     else if (left_incentive >= right_incentive && left_incentive >= front_incentive) direction = LEFT;
     else direction = RIGHT;
     
     // correct for bad planned turns
     printf("danger sum [left, front, right] = [%.2f, %.2f, %.2f]\n", left_danger_sum, front_danger_sum, right_danger_sum);
     int front_dangerous = front_danger_sum > params->zombie_threshold || front_obstacle_sum > params->obstacle_threshold;
     int right_dangerous = right_danger_sum > params->zombie_threshold || right_obstacle_sum > params->obstacle_threshold;
     int left_dangerous = left_danger_sum > params->zombie_threshold || left_obstacle_sum > params->obstacle_threshold;

     if ((direction == LEFT && left_dangerous)|| (direction == RIGHT && right_dangerous)) {
        printf("MAKING CORRECTION\n");
        if (!front_dangerous) {
            direction = FORWARD;
        } else if (!right_dangerous) {
            direction = RIGHT;
        } else if (!left_dangerous) {
            direction = LEFT;
        } else {
            if (front_obstacle_sum < params->obstacle_threshold) {
                direction = FORWARD;
            } else {
                float lr_obstacles[] = {left_obstacle_sum, right_obstacle_sum};
                direction = getIndexOfMin(lr_obstacles, 2);
            }
        }
     }

    // printf("DIRECTION: %d\n", direction);
    return direction;
}

void robot_control(int timer, Control *params)
{
    // Variables dictating image processing
    int viewpanes_vertical = 3; // no. of vertical panes to split view
    int viewpanes_horizontal = 2; // no. of horizontal panes to split view

    int front_image_width = wb_camera_get_width(4); // width of camera 4, $300
    int front_image_height = wb_camera_get_height(4); // height of camera 4, $300

    int right_image_width = wb_camera_get_width(9); // width of camera 9, $200
    int right_image_height = wb_camera_get_height(9); // height of camera 9, $200

    int left_image_width = wb_camera_get_width(10); // width of camera 10, $200
    int left_image_height = wb_camera_get_height(10); // height of camera 10, $200

    const double *gps = wb_gps_get_values(2); // get gps coordinates, $300
    if (timer % 16 == 0) {
        // print sensor values
        printf("\n");
        // printf("GPS:  [ x y z ] = [ %+.3f %+.3f %+.3f ]\n", gps[0], gps[1], gps[2]);

        int direction = analyze_cameras(params, viewpanes_vertical, viewpanes_horizontal, front_image_width, front_image_height,
                                    right_image_width, right_image_height, left_image_width, left_image_height);

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
    
    // update gps history
    params->last_gps[0] = (double)gps[0];
    params->last_gps[1] = (double)gps[1];
    params->last_gps[2] = (double)gps[2];
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{

  struct Robot robot_info = {100,100};
  wb_robot_init();

  base_init();
  arm_init();
  gripper_init();
  passive_wait(0.1);

  display_helper_message();

  int pc = 0;
  wb_keyboard_enable(TIME_STEP);
  int timer = 0;

  WbNodeRef robot_node = wb_supervisor_node_get_from_def("Youbot");
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");

  get_all_berry_pos();

  int robot_not_dead = 1;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////// CHANGE CODE BELOW HERE ONLY ////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////

  // sensor choices
  
//  wb_accelerometer_enable(1,1);     
  wb_gps_enable(2,TIME_STEP);          
//  wb_compass_enable(3,TIME_STEP);      
  wb_camera_enable(4,TIME_STEP);      
//  wb_camera_enable(5,TIME_STEP);
//  wb_camera_enable(6,TIME_STEP);
//  wb_camera_enable(7,TIME_STEP);
//  wb_camera_enable(8,TIME_STEP);
  wb_camera_enable(9,TIME_STEP);
  wb_camera_enable(10,TIME_STEP);
//  wb_camera_enable(11,TIME_STEP);
//  wb_gyro_enable(12,TIME_STEP);         
//  wb_light_sensor_enable(13,TIME_STEP);
//  wb_receiver_enable(14,TIME_STEP);
//  wb_range_finder_enable(15,TIME_STEP);
//  wb_lidar_enable(16,1);                

  // Robot info structs
  struct Robot last_info = {100,100};
  struct Robot current_info = {100,100};

  // Parameters fed to control function
  Control* parameters = malloc(sizeof(Control));
  parameters->last_gps = malloc(sizeof(double) * 3);
  
  parameters->last_gps[0] = 0;
  parameters->last_gps[1] = 0;
  parameters->last_gps[2] = 0;
  
  parameters->turning = 0;
  parameters->timesteps = 0;
  parameters->stuck_steps = 0;
  parameters->time = 0;
  
  parameters->zombie_threshold = 20.0;
  parameters->berry_threshold = 0.5;
  parameters->obstacle_threshold = 15.0;
  
  parameters->zombie_sensitivity= 0.65;
  parameters->berry_sensitivity = 0.15;
  parameters->obstacle_sensitivity = 0.10;
  
  parameters->last_info = last_info;
  parameters->current_info = current_info;
  parameters->q = queue_constuct();


  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  while (robot_not_dead == 1)
  {
    if (robot_info.health < 0)
    {
        robot_not_dead = 0;
        printf("ROBOT IS OUT OF HEALTH\n");
    }

    if (timer % 2 == 0)
    {
        const double *trans = wb_supervisor_field_get_sf_vec3f(trans_field);
        check_berry_collision(&robot_info, trans[0], trans[2]);
        check_zombie_collision(&robot_info, trans[0], trans[2]);
    }
    if (timer == 16)
    {
        update_robot(&robot_info);
        timer = 0;
    }
    step();
    int c = keyboard(pc);
    pc = c;
    timer=timer+1;


    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////// CHANGE CODE BELOW HERE ONLY ////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // control function called every time step
    
    parameters->current_info.health = robot_info.health;
    parameters->current_info.energy = robot_info.energy;

    robot_control(timer, parameters);
    
    parameters->last_info.health = robot_info.health;
    parameters->last_info.energy = robot_info.energy;
    
    (parameters->time)++;

    if (parameters->time > 100000000000){
      parameters->time = 0;
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
  }
  free(parameters->last_gps);
  queue_destroy(parameters->q);
  free(parameters);
  wb_robot_cleanup();

  return 0;
}