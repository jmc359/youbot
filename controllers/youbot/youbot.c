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

void rotate_robot(int angle)
{
    WbNodeRef robot_node = wb_supervisor_node_get_from_def("Youbot");
    WbFieldRef rot_field = wb_supervisor_node_get_field(robot_node, "rotation");
    double rotation[4];
    if (angle == 0) { rotation[0] = 1; rotation[1] = 0; rotation[2] = 0; rotation[3] = -1.57; }
    if (angle == 15) { rotation[0] = -0.985; rotation[1] = 0.126; rotation[2] = 0.122; rotation[3] = 1.59; }
    if (angle == 30) { rotation[0] = -0.938; rotation[1] = 0.247; rotation[2] = 0.244; rotation[3] = 1.63; }
    if (angle == 45) { rotation[0] = -0.866; rotation[1] = 0.355; rotation[2] = 0.352; rotation[3] = 1.71; }
    if (angle == 60) { rotation[0] = -0.778; rotation[1] = 0.445; rotation[2] = 0.443; rotation[3] = 1.82; }
    if (angle == 75) { rotation[0] = -0.681; rotation[1] = 0.519; rotation[2] = 0.516; rotation[3] = 1.94; }
    if (angle == 90) { rotation[0] = -0.581; rotation[1] = 0.577; rotation[2] = 0.572; rotation[3] = 2.09; }
    if (angle == 105) { rotation[0] = -0.48; rotation[1] = 0.621; rotation[2] = 0.619; rotation[3] = 2.24; }
    if (angle == 120) { rotation[0] = -0.381; rotation[1] = 0.654; rotation[2] = 0.653; rotation[3] = 2.41; }
    if (angle == 135) { rotation[0] = -0.284; rotation[1] = 0.679; rotation[2] = 0.677; rotation[3] = 2.58; }
    if (angle == 150) { rotation[0] = -0.189; rotation[1] = 0.695; rotation[2] = 0.694; rotation[3] = 2.76; }
    if (angle == 165) { rotation[0] = -0.095; rotation[1] = 0.704; rotation[2] = 0.704; rotation[3] = 2.95; }
    if (angle == 180) { rotation[0] = -0.00268959; rotation[1] = 0.707126; rotation[2] = 0.707083; rotation[3] = 3.13102; }
    if (angle == 195) { rotation[0] = 0.090; rotation[1] = 0.704; rotation[2] = 0.704; rotation[3] = -2.97; }
    if (angle == 210) { rotation[0] = 0.183; rotation[1] = 0.695; rotation[2] = 0.695; rotation[3] = -2.78; }
    if (angle == 225) { rotation[0] = 0.278; rotation[1] = 0.679; rotation[2] = 0.680; rotation[3] = -2.6; }
    if (angle == 240) { rotation[0] = 0.375; rotation[1] = 0.655; rotation[2] = 0.656; rotation[3] = -2.43; }
    if (angle == 255) { rotation[0] = 0.473; rotation[1] = 0.622; rotation[2] = 0.624; rotation[3] = -2.26; }
    if (angle == 270) { rotation[0] = 0.574; rotation[1] = 0.578; rotation[2] = 0.580; rotation[3] = -2.10; }
    if (angle == 285) { rotation[0] = 0.674; rotation[1] = 0.521; rotation[2] = 0.524; rotation[3] = -1.96; }
    if (angle == 300) { rotation[0] = 0.771; rotation[1] = 0.449; rotation[2] = 0.452; rotation[3] = -1.83; }
    if (angle == 315) { rotation[0] = 0.860; rotation[1] = 0.360; rotation[2] = 0.363; rotation[3] = -1.72; }
    if (angle == 330) { rotation[0] = 0.933; rotation[1] = 0.254; rotation[2] = 0.257; rotation[3] = -1.64; }
    if (angle == 345) { rotation[0] = 0.982; rotation[1] = 0.133; rotation[2] = 0.137; rotation[3] = -1.59; }
    wb_supervisor_field_set_sf_rotation(rot_field,rotation);
    robot_angle = angle;	
}


void go_forward()
{
	base_forwards();
}

void stop()
{
	base_reset();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// CHANGE CODE BELOW HERE ONLY ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

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

void print2DArray(float *array, int m, int n) 
{ 
    int i, j; 
    for (i = 0; i < m; i++) 
      for (j = 0; j < n; j++) 
        printf("%f ", *((array+i*n) + j)); 
} 

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

void robot_control(int timer)
{
    ////////////// TO ROTATE THE ROBOT (BETWEEN 0 - 345) WITH 15 DEGREE INTERVALS ///////////////
    //rotate_robot(45);
    //rotate_robot(255);
    /////////////////////////////////////////////////////////////////////////////////////////////
  
    ////////////// TO MOVE ROBOT FORWARD AND TO STOP IT /////////////////////////////////////////
    go_forward();
    // stop();
    /////////////////////////////////////////////////////////////////////////////////////////////
    // printf("\f"); // clear console
    
    int viewpanes_vertical = 3; // no. of vertical panes to split view
    int viewpanes_horizontal = 2; // no. of horizontal panes to split view
    int image_width = 128; // standard image width
    int image_height = 64; // standard image height
    float viewpanes[viewpanes_vertical][viewpanes_horizontal];

    
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

  //testing receiver -- see main
  //WbDeviceTag rec = wb_robot_get_device("receiver");
    
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
    robot_control(timer);
    //go_forward();
    //stop();

    /* // testing receiver
    int count = 0;
    if (wb_receiver_get_queue_length(rec) > 0) 
    {
        const char *buffer = wb_receiver_get_data(rec);
        printf("Communicating: received \"%s\"\n", buffer);
    	 wb_receiver_next_packet(rec);
    	 count++;
    } printf("Receiver: %d zombies\n", count); */
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
  }

  wb_robot_cleanup();

  return 0;
}
