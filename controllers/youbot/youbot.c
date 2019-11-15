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

<<<<<<< HEAD
=======
//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// ONLY USE THE FOLLOWING FUNCTIONS TO MOVE THE ROBOT /////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////


>>>>>>> 82853d782851f4f9470ca34e468a55b80416c959
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
<<<<<<< HEAD
{
  base_turn_left();
  robot_angle = robot_angle + 90;
  if (robot_angle == 360)
    robot_angle = 0;

}

void turn_right()
{
=======
{
  base_turn_left();
  robot_angle = robot_angle + 90;
  if (robot_angle == 360)
    robot_angle = 0;

}

void turn_right()
{
>>>>>>> 82853d782851f4f9470ca34e468a55b80416c959
  base_turn_right();  
  robot_angle = robot_angle - 90;
  if (robot_angle == -90)
    robot_angle = 270;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// CHANGE CODE BELOW HERE ONLY ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// directions for turning/translating
#define LEFT (0)
#define RIGHT (1)
#define FORWARD (2)
#define BACKWARD (3)

// logic for determining how much zombie in frame
// could also be used for determining 'usefulness'
// considering berries
float calcZombiness(int g, int b){
  return pow(((g + b)/(255*2.0)), 10) * 100;
}

// returns index of minimum int in @array
int getIndexOfMin(float* array, size_t size){
  int minimum = 0;
   
  for (int i = 1; i < size; i++)
  {
      if (array[i] < array[minimum])
         minimum = i;
  }
  return minimum;
} // test // int temp[] = {6, 43, 2, 1, 4}; printf("minimum value %d\n", getIndexOfMin(temp, (sizeof(temp)/sizeof(temp[0]))));   

double get_bearing_in_degrees() {
  const double *north = wb_compass_get_values(3);
  double rad = atan2(north[0], north[2]);
  double bearing = (rad - 1.5708) / M_PI * 180.0;
  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}

double get_bearing_in_radians() {
  const double *north = wb_compass_get_values(3);
  return atan2(north[0], north[2]);
}

int** color_mask(char *image, int color_min[], int color_max[], int image_width, int image_height){
    int ** color_mask_img[image_width][image_height];
    
    for (int x = 0; x < image_width; x++) {
      for (int y = 0; y < image_height; y++) {
        int r = wb_camera_image_get_red(image, image_width, x, y);
        int g = wb_camera_image_get_green(image, image_width, x, y);
        int b = wb_camera_image_get_blue(image, image_width, x, y);
        
        int r_in_color_range = (r > color_min[0]) && (r <= color_max[0]);
        int g_in_color_range = (g > color_min[1]) && (g <= color_max[1]);
        int b_in_color_range = (b > color_min[2]) && (b <= color_max[2]);
        
        int good_color = r_in_color_range && g_in_color_range && b_in_color_range;
        color_mask_img[x][y] = good_color; // ?? what does this mean? 
        
        if (good_color == 1) { // ?? so checking if entire image is void of certain color? 
          printf("Good color\n");
        }
      }
    }
    
    return color_mask_img;
}

// turning the robot
void turn(int direction, int *turning, int *timesteps){
  if ((*timesteps) == 0){
    stop();
    (*turning) = 1;
    printf("TURNING: %d\n", direction);
    switch(direction){
      case LEFT:  turn_left(); break;
      case RIGHT: turn_right(); break;
    }
  }
}

// update timestep counter for turning
void turn_update(int *turning, int *timesteps){
  if ((*turning)){
    if ((*timesteps) < 150){
      (*timesteps)++;
    }
    else{
      (*turning) = 0;
      (*timesteps) = 0;
    }
  }
}

// translating forward/backward
void translate(int direction, int *turning){
  if(!(*turning)){
    switch(direction){
      case FORWARD:  go_forward(); break;
      case BACKWARD: go_backward(); break;
    }
  }
}

<<<<<<< HEAD
=======
int** color_mask(char *image, int color_min[], int color_max[], int image_width, int image_height){
    int ** color_mask_img[image_width][image_height];
    
    for (int x = 0; x < image_width; x++) {
      for (int y = 0; y < image_height; y++) {
        int r = wb_camera_image_get_red(image, image_width, x, y);
        int g = wb_camera_image_get_green(image, image_width, x, y);
        int b = wb_camera_image_get_blue(image, image_width, x, y);
        
        int r_in_color_range = (r > color_min[0]) && (r <= color_max[0]);
        int g_in_color_range = (g > color_min[1]) && (g <= color_max[1]);
        int b_in_color_range = (b > color_min[2]) && (b <= color_max[2]);
        
        int good_color = r_in_color_range && g_in_color_range && b_in_color_range;
        color_mask_img[x][y] = good_color;
        
        if (good_color == 1) {
          printf("Good color\n");
        }
      }
    }
    
    return color_mask_img;
}

// Turning the robot
void turn(int direction, int *turning, int *timesteps){
  if ((*timesteps) == 0){
    stop();
    (*turning) = 1;
    printf("TURNING: %d\n", direction);
    switch(direction){
      case LEFT:  turn_left(); break;
      case RIGHT: turn_right(); break;
    }
  }
}

// Update timestep counter for turning
void turn_update(int *turning, int *timesteps){
  if ((*turning)){
    if ((*timesteps) < 150){
      (*timesteps)++;
    }
    else{
      (*turning) = 0;
      (*timesteps) = 0;
    }
  }
}

// Translating forward/backward
void translate(int direction, int *turning){
  if(!(*turning)){
    printf("TRANSLATING\n");
    switch(direction){
      case FORWARD:  go_forward(); break;
      case BACKWARD: go_backward(); break;
    }
  }
}
>>>>>>> 82853d782851f4f9470ca34e468a55b80416c959

void robot_control(int timer)
{
    ////////////// TO ROTATE THE ROBOT (BETWEEN 0 - 345) WITH 15 DEGREE INTERVALS ///////////////
    //rotate_robot(45);
    //rotate_robot(255);
    /////////////////////////////////////////////////////////////////////////////////////////////
  
    ////////////// TO MOVE ROBOT FORWARD AND TO STOP IT /////////////////////////////////////////
    // go_forward();
    // stop();
    /////////////////////////////////////////////////////////////////////////////////////////////
    
    int viewpanes_vertical = 3; // no. of vertical panes to split view
    int viewpanes_horizontal = 2; // no. of horizontal panes to split view
    int image_width = 128; // standard image width
    int image_height = 64; // standard image height
    float viewpanes[viewpanes_vertical][viewpanes_horizontal];


    // Color mask values
    const int blue_color_min[3] = {9, 38, 93};
    const int blue_color_max[3] = {28, 111, 198};
    
    const int aqua_color_min[3] = {11, 67, 68};
    const int aqua_color_max[3] = {76, 192, 175};
    
    const int green_color_min[3] = {10,52,16};
    const int green_color_max[3] = {64, 158, 68};
    
    const int purple_color_min[3] = {45, 24, 104};
    const int purple_color_max[3] = {183, 82, 249};
    
    const int red_color_min[3] = {76, 18, 31};
    const int red_color_max[3] = {209, 62, 44};
<<<<<<< HEAD
    
=======

>>>>>>> 82853d782851f4f9470ca34e468a55b80416c959
    
    if (timer % 16 == 0) { // n % 16 (different camera parameters now)
        
        // get GPS values
        const double *values = wb_gps_get_values(2);
        printf("GPS:  [ x y z ] = [ %+.3f %+.3f %+.3f ]\n", values[0], values[1], values[2]);
        
        // get accelerometer values
        const double *acceleration = wb_accelerometer_get_values(1);
        printf("ACCL: [ x y z ] = [ %+.3f %+.3f %+.3f ]\n", acceleration[0], acceleration[1], acceleration[2]);
        
        // get compass values
        printf("COMP: radians = %+.3f, degrees = %+.3f\n", get_bearing_in_radians(), get_bearing_in_degrees());
        
        // get gyro rotation axes
        const double *vel = wb_gyro_get_values(12);
        printf("GYRO: [ x y z ] = [ %+.3f %+.3f %+.3f ]\n", vel[0], vel[1], vel[2]);
        
        // RR ?? get light value (aka interpolated irradiance with ?lookupTable)
        const double light = wb_light_sensor_get_value(13);
        printf("LGHT: %.3f\n", light);
        
        
        // get image from camera
        const unsigned char *image = wb_camera_get_image(6);
        
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
            int r_avg = r_sum / total_pix;
            int g_avg = g_sum / total_pix;
            int b_avg = b_sum / total_pix;
              
            r_sum = 0; g_sum = 0; b_sum = 0; // reset sums
            
            // store 'zombieness' computation
            viewpanes[vx][vy] = calcZombiness(g_avg, b_avg);
            printf("V_V=%d [start=%3d, end=%3d]; V_H=%d [start=%3d, end=%3d]; zombieness=%f\n", vx, start_vx, end_vx, vy, start_vy, end_vy, viewpanes[vx][vy]);
          }
          
        }
        
        // get vertical panes only
        float views_vertical[viewpanes_vertical];
        for (int row = 0; row < viewpanes_vertical; row++){
          float sum = 0;
          for (int col = 0; col < viewpanes_horizontal; col++){
            sum += viewpanes[row][col];
          }
          views_vertical[row] = sum;
        }
        
        // compute safest route,
        int safest_pane = getIndexOfMin(views_vertical, (sizeof(views_vertical)/sizeof(views_vertical[0])));
        double safest_direction = round(((float)safest_pane/(viewpanes_vertical - 1)) * 2)/2;
        printf("safest pane = %d, safest direction = %.1f\n", safest_pane, safest_direction);
        
        // determine threshold for turning
        // example: when zombieness > 1
        
        // make turn
        
        // create blue zombie color mask
        int** color_mask_img = color_mask(image, blue_color_min, blue_color_max, image_width, image_height);
<<<<<<< HEAD
        
=======
>>>>>>> 82853d782851f4f9470ca34e468a55b80416c959
    }
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

  int turning = 0;
  int timesteps = 0;
  int direction = 0;
  int threshold = 0;

  //testing receiver -- see main
  //WbDeviceTag rec = wb_robot_get_device("receiver");
    
  int turning = 0;
  int timesteps = 0;
  int direction = 0;
  int threshold = 0;
  
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
    
    // this is called everytime step.
<<<<<<< HEAD
    robot_control(timer);
    /*translate(FORWARD, &turning);
=======
    // robot_control(timer);
    translate(FORWARD, &turning);
>>>>>>> 82853d782851f4f9470ca34e468a55b80416c959
    if(threshold > 200){
      printf("Threshold: %d\n", threshold);
      turn(direction % 2, &turning, &timesteps);
      direction++;
      threshold = 0;
    }
    threshold++;
<<<<<<< HEAD
    turn_update(&turning, &timesteps);*/
=======
    turn_update(&turning, &timesteps);
>>>>>>> 82853d782851f4f9470ca34e468a55b80416c959
    //go_forward();
    //stop();
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
  }

  wb_robot_cleanup();

  return 0;
}
