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
 * Helper functions for printing/creating/returning min of arrays
 */

// prints 2d float array
void print2DArray(int *array, int m, int n) 
{ 
    int i, j; 
    for (i = 0; i < m; i++) 
      for (j = 0; j < n; j++) 
        printf("%d ", *((array+i*n) + j)); 
} 

// creates 2d float array
float** create2DArray(int c, int r)
{
    float* values = calloc(c * r, sizeof(float));
    float** rows = malloc(r * sizeof(float*));
    for (int i = 0; i < r; ++i)
    {
        rows[i] = values + i * c;
    }
    return rows;
}

// creates 3d int array
int*** create3DArray(int numRows, int numCols, int numLevels)
{
    int ***levels;
    levels = malloc(numLevels *sizeof(int *)); //Contains all levels

    int rowIndex, levelIndex;

    for (levelIndex = 0; levelIndex < numLevels; levelIndex++)
    {
        int **level = malloc(numRows * sizeof(int *)); //Contains all rows

        for(rowIndex = 0; rowIndex < numRows; rowIndex++)
        {
            level[rowIndex] = malloc(numCols * sizeof(int)); //Contains all columns
        }      

        levels[levelIndex] = level;
    }

    return levels;
}

// returns index of minimum int in @array
int getIndexOfMin(float* array, int size){
  int minimum = 0;

  for (int i = 0; i < size; i++)
  {
      if (array[i] < array[minimum])
         minimum = i;
  }
  return minimum;
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
void rotate(int direction, int *turning, int *timesteps){
  if ((*timesteps) == 0){ // Initiate turn at first timestep
    stop();
    (*turning) = 1;
    switch(direction){
      case LEFT:  turn_right(); break; // **original functions are swapped**
      case RIGHT: turn_left(); break;
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

static int min_colors[8][3] = {{10, 39, 97}, {11, 67, 68}, {10, 52, 16}, {45, 18, 92},
                               {76, 18, 31}, {62, 38, 67}, {62, 38, 32}, {69, 67, 13}};
static int max_colors[8][3] = {{28, 111, 198}, {76, 192, 175}, {64, 158, 68}, {115, 49, 185},
                               {209, 62, 44}, {193, 124, 167}, {194, 124, 85}, {207, 195, 37}};

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
int in_range(double *hsv, double *hsv_compare, double epsilon)
{
    int h_valid = abs((int)hsv_compare[0] - (int)hsv[0])%360 < epsilon;    
    int s_valid = abs(hsv_compare[1] - hsv[1]) < epsilon;    
    int v_valid = abs(hsv_compare[2] - hsv[2]) < epsilon;

    int color_in_range = (h_valid) && (s_valid) && (v_valid);
    return color_in_range;
}

// function for counting matching colored pixels in image given input color range and rgb image
int color_mask_count(const unsigned char *image, int color, int image_width, int image_height, int save_mask) {
    double epsilon = 5.0;
    
    double hsv_min[3];
    rgb_to_hsv(min_colors[color], hsv_min);
    double hsv_max[3];
    rgb_to_hsv(max_colors[color], hsv_max);

    int mask_pixels = 0;
    
    FILE *file_pointer = NULL;
    if (save_mask) {
      file_pointer = fopen("image_mask.txt", "w");
    }
    
    // fprintf(file_pointer, "%d,%d", image_height, image_width);
    
    for (int x = 0; x < image_width; x++) {
        for (int y = 0; y < image_height; y++) {
            int r = wb_camera_image_get_red(image, image_width, x, y);
            int g = wb_camera_image_get_green(image, image_width, x, y);
            int b = wb_camera_image_get_blue(image, image_width, x, y);
    
            int rgb[3] = {r, g, b};
            double hsv[3];
            rgb_to_hsv(rgb, hsv);
            
            int in_range_min = in_range(hsv, hsv_min, epsilon);
            int in_range_max = in_range(hsv, hsv_max, epsilon);
            int found_color = in_range_min || in_range_max;
            mask_pixels += found_color;
            
            if (save_mask) {
                if (y == 0) {
                    fprintf(file_pointer, "%d", found_color);
        
                } else {
                    fprintf(file_pointer, ",%d", found_color);
                }
            }
        }
        if (save_mask) {
          fprintf(file_pointer, "\n");
        }
    }
    
    
    if (save_mask) {
      fclose(file_pointer);
    }
    return mask_pixels;
}

void color_mask_image(const unsigned char *image, int color, int image_width, int image_height, int mask_image[image_width][image_height]) {
    double epsilon = 5.0;
    
    double hsv_min[3];
    rgb_to_hsv(min_colors[color], hsv_min);
    double hsv_max[3];
    rgb_to_hsv(max_colors[color], hsv_max);
       
    for (int x = 0; x < image_width; x++) {
        for (int y = 0; y < image_height; y++) {
            int r = wb_camera_image_get_red(image, image_width, x, y);
            int g = wb_camera_image_get_green(image, image_width, x, y);
            int b = wb_camera_image_get_blue(image, image_width, x, y);
    
            int rgb[3] = {r, g, b};
            double hsv[3];
            rgb_to_hsv(rgb, hsv);
            
            int in_range_min = in_range(hsv, hsv_min, epsilon);
            int in_range_max = in_range(hsv, hsv_max, epsilon);
            int found_color = in_range_min || in_range_max;
            if (found_color != 1 && found_color != 0) {
                printf("Found color {%d, %d}: %d", x, y, found_color);
            }
            mask_image[x][y] = found_color;
            
            if (mask_image[x][y] != 1 && mask_image[x][y] != 0) {
                printf("Not matching - {%d, %d}", x, y);
            }
        }
    }
}

// logic for determining how much zombie in frame
// could also be used for determining 'usefulness'
// considering berries
float calcZombiness(int g, int b){
  return pow(((g + b)/(255*2.0)), 10) * 100;
}

// return vertical panes from image
float *get_views_vertical(const unsigned char *image, int viewpanes_vertical, int viewpanes_horizontal, int image_width, int image_height){
  float viewpanes[viewpanes_vertical][viewpanes_horizontal];
  for (int vx = 0; vx < viewpanes_vertical; vx++){
    for (int vy = 0; vy < viewpanes_horizontal; vy++){
      // define start and end for current vertical pane
      int start_vx = (image_width/viewpanes_vertical) * vx;
      int end_vx = (image_width/viewpanes_vertical) * (vx + 1);
      
      // define start and end for current horizontal pane
      int start_vy = (image_height/viewpanes_horizontal) * vy;
      int end_vy = (image_height/viewpanes_horizontal) * (vy + 1);
    
      // get rgb values of current pane
      int r_sum = 0, g_sum = 0, b_sum = 0;
      for (int x = start_vx; x < end_vx; x++){
        for (int y = start_vy; y < end_vy; y++){
          // compute rgb values of current pixel
          r_sum += wb_camera_image_get_red(image, image_width, x, y);
          g_sum += wb_camera_image_get_green(image, image_width, x, y);
          b_sum += wb_camera_image_get_blue(image, image_width, x, y);
        }
      }
      
      // compute average rgb of current pane
      int total_pix = (end_vx - start_vx) * (end_vy - start_vy);
      // int r_avg = r_sum / total_pix;
      int g_avg = g_sum / total_pix;
      int b_avg = b_sum / total_pix;
        
      r_sum = 0; g_sum = 0; b_sum = 0; // reset sums
      
      // store 'zombieness' computation
      viewpanes[vx][vy] = calcZombiness(g_avg, b_avg);
      printf("V_V=%d [start=%3d, end=%3d]; V_H=%d [start=%3d, end=%3d]; zombieness=%f\n", vx, start_vx, end_vx, vy, start_vy, end_vy, viewpanes[vx][vy]);
    }
  }
  
  // get vertical panes only
  float *views_vertical = malloc(sizeof(float) * viewpanes_vertical);
  // float views_vertical[viewpanes_vertical];
  for (int row = 0; row < viewpanes_vertical; row++){
    float sum = 0;
    for (int col = 0; col < viewpanes_horizontal; col++){
      sum += viewpanes[row][col];
    }
    views_vertical[row] = sum;
  }
  return views_vertical;
}

// return vertical panes from image
float *get_views_vertical_mask(int viewpanes_vertical, int viewpanes_horizontal, int image_width, int image_height, int mask_array[image_width][image_height]){
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
            // if (mask_sum != 1 && mask_sum != 0) {
                // printf("mask_array[%d][%d]: %d\n", x, y, mask_array[x][y]);
            // }
        }
      }
      
      // compute average rgb of current pane
      int total_pix = (end_vx - start_vx) * (end_vy - start_vy);
      float mask_avg = (float)mask_sum / total_pix;
      printf("V_V=%d [start=%3d, end=%3d]; V_H=%d [start=%3d, end=%3d]; mask sum=%d; zombieness=%f\n", vx, start_vx, end_vx, vy, start_vy, end_vy, mask_sum, mask_avg);
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
    for (int col = 0; col < viewpanes_horizontal; col++){
      sum += viewpanes[row][col];
    }
    views_vertical[row] = sum;
  }
  return views_vertical;
}

// RR - this need more testing... 
float sumOfArray(float a[], int n) {
  float sum = 0;

  for(int i = 0; i < n; i++) {
    sum += a[i];
    printf("@i=%d, a[i]=%f, sum=%f\n", i, a[i], sum); // hm... uncomment this and you get a different sum...
  }
  
  return sum;
}



/*
 * Main robot control function, called every time step
 */
void robot_control(int timer, int *turning, int *timesteps, float threshold,
                   struct Robot last, struct Robot current)
{   
    // Variables dictating image processing
    int viewpanes_vertical = 3; // no. of vertical panes to split view
    int viewpanes_horizontal = 2; // no. of horizontal panes to split view
    int image_width = 256; // standard image width
    int image_height = 128; // standard image height

    
    if (timer % 16 == 0) { // n % 16 (different camera parameters now)
        // get sensor values
        const double *values = wb_gps_get_values(2);
        const double *acceleration = wb_accelerometer_get_values(1);
        const double *vel = wb_gyro_get_values(12);
        const double light = wb_light_sensor_get_value(13);
        const unsigned char *image = wb_camera_get_image(6);
        
        // Print # of pixels matching color
        // int mask_pixels = color_mask_count(image, ORANGE, image_width, image_height, 1);
        // printf("Mask pixels: %d\n", mask_pixels);
        int mask_image[image_width][image_height];
        color_mask_image(image, BLUE, image_width, image_height, mask_image);
        
        // print sensor values
        printf("GPS:  [ x y z ] = [ %+.3f %+.3f %+.3f ]\n", values[0], values[1], values[2]);
        printf("ACCL: [ x y z ] = [ %+.3f %+.3f %+.3f ]\n", acceleration[0], acceleration[1], acceleration[2]);
        printf("COMP: radians = %+.3f, degrees = %+.3f\n", get_bearing_in_radians(), get_bearing_in_degrees());
        printf("GYRO: [ x y z ] = [ %+.3f %+.3f %+.3f ]\n", vel[0], vel[1], vel[2]);
        printf("LGHT: %.3f\n", light); // RR ?? get light value (aka interpolated irradiance with ?lookupTable)
        
        // compute safest route, direction
        // float *views_vertical = get_views_vertical(image,viewpanes_vertical,viewpanes_horizontal,image_width,image_height);
        float *views_vertical = get_views_vertical_mask(viewpanes_vertical,viewpanes_horizontal,image_width,image_height, mask_image);
        int safest_pane = getIndexOfMin(views_vertical, viewpanes_vertical); // (sizeof(views_vertical)/sizeof(views_vertical[0])));
        double safest_direction = round(((float)safest_pane/(viewpanes_vertical - 1)) * 2)/2;
        float zombieness = sumOfArray(&views_vertical[safest_pane], (sizeof(views_vertical)/sizeof(views_vertical[0])));
        printf("safest pane = %d, safest direction = %.1f, zombieness = %.4f\n", safest_pane, safest_direction, zombieness);
        free(views_vertical);
        
        // make turn
        printf("checking thresholds\n");
        if (zombieness > threshold) {
          if (safest_direction == 0){
            printf("ROTATING LEFT\n");
            rotate(LEFT, turning, timesteps);
          }
          else if (safest_direction == 1){
            printf("ROTATING RIGHT\n");
            rotate(RIGHT, turning, timesteps);
          }
        }
    }
    translate(FORWARD, turning);
    rotate_update(turning, timesteps);
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
  wb_accelerometer_enable(1,1);         // test in robot_control
  wb_gps_enable(2,TIME_STEP);           // test in robot_control
  wb_compass_enable(3,TIME_STEP);       // test in robot_control
  wb_camera_enable(4,TIME_STEP);        // test in robot_control
  wb_camera_enable(5,TIME_STEP);
  wb_camera_enable(6,TIME_STEP);
  wb_camera_enable(7,TIME_STEP);
  wb_camera_enable(8,TIME_STEP);
  wb_camera_enable(9,TIME_STEP);
  wb_camera_enable(10,TIME_STEP);
  wb_camera_enable(11,TIME_STEP);
  wb_gyro_enable(12,TIME_STEP);         // test in robot_control
  wb_light_sensor_enable(13,TIME_STEP); // test in robot control
  wb_receiver_enable(14,TIME_STEP);     // test in main
  wb_range_finder_enable(15,TIME_STEP);
  wb_lidar_enable(16,1);                // RR ?? not fully tested - see main
  
  // RR ?? testing lidar -- what about testing info extraction from point cloud? 
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  wb_lidar_enable_point_cloud(lidar);

  //testing receiver -- see main
  //WbDeviceTag rec = wb_robot_get_device("receiver");

  // Variables dictating turning
  int turning = 0; // bool for turning
  int timesteps = 0; // how many timesteps into current turn
  float threshold = 0.5; // threshold for initiating turn

  // robot health and info
  struct Robot last_info = {100,100};
    
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
    robot_control(timer, &turning, &timesteps, threshold, last_info, robot_info);
    last_info.health = robot_info.health;
    last_info.energy = robot_info.energy;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
  }

  wb_robot_cleanup();

  return 0;
}
