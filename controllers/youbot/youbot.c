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


int getTimer(int timer){
   return timer;
}

// logic for determining how much zombie(s)
// might change per design sessions
int calcZombiness(int g, int b){
  return (g + b);
}

// returns index of minimum int in @array
int getIndexOfMin(int* array, size_t size){
  int minimum = 0;
   
  for (int i = 1; i < size; i++)
  {
      if (array[i] < array[minimum])
         minimum = i;
  }
  return minimum;
} // test // int temp[] = {6, 43, 2, 1, 4}; printf("minimum value %d\n", getIndexOfMin(temp, (sizeof(temp)/sizeof(temp[0]))));
    

void robot_control(int timer)
{
  ////////////// TO ROTATE THE ROBOT (BETWEEN 0 - 345) WITH 15 DEGREE INTERVALS ///////////////
  //rotate_robot(45);
  //rotate_robot(255);
  /////////////////////////////////////////////////////////////////////////////////////////////
  
  ////////////// TO MOVE ROBOT FORWARD AND TO STOP IT /////////////////////////////////////////
       //go_forward();
  // stop();
  /////////////////////////////////////////////////////////////////////////////////////////////
    
    go_forward();
    
    int viewpanes = 3; // number of panes to split view
    int image_width = 128; // standard image width
    int image_height = 64; // standard image height
    int view_colors[viewpanes];

    
    if (timer % 16 == 0) { // n % 16 (different camera parameters now)
        // Get GPS values
        const double *values = wb_gps_get_values(2);
        printf("GPS X: %f  Y: %f  Z: %f\n", values[0], values[1], values[2]);
        
        // Get image
        const unsigned char *image = wb_camera_get_image(4);
        for (int i = 0; i < viewpanes; i++) {
            int viewfactor = image_width/viewpanes; // loss is negligent
            int r_sum = 0, g_sum = 0, b_sum = 0;
            for (int x = (viewfactor * i); // start pixel of current pane
                     x < (viewfactor * (i + 1)); // start pixel of next pane
                     x++)
            {
                for (int y = 0; y < image_height; y++) 
                {
                    r_sum += wb_camera_image_get_red(image, image_width, x, y);
                    g_sum += wb_camera_image_get_green(image, image_width, x, y);
                    b_sum += wb_camera_image_get_blue(image, image_width, x, y);
                }
            //printf("x=%d, x_start=%d, x_end=%d\n", x, (viewfactor * i), (viewfactor * (i + 1)));
            }
            int r_avg = r_sum / (image_width * image_height);
            int g_avg = g_sum / (image_width * image_height);
            int b_avg = b_sum / (image_width * image_height);
             //printf("viewpane=%d: red=%d, green=%d, blue=%d\n", (i + 1), r_avg, g_avg, b_avg);
            printf("viewpane=%d; zombieness=%d\n", (i + 1), calcZombiness(g_avg, b_avg));
            view_colors[i] = calcZombiness(g_avg, b_avg);
        }
        
        // compute safest route
        int safe_pane = getIndexOfMin(view_colors, (sizeof(view_colors)/sizeof(view_colors[0])));
        printf("safest pane=%d\n", safe_pane);
        
        // next steps -- convert safest pane into angle, turn by angle --> move forward. 
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
    
  // wb_accelerometer_enable(1,1);
  wb_gps_enable(2,TIME_STEP);
  wb_compass_enable(3,TIME_STEP);
  wb_camera_enable(4,TIME_STEP);
  wb_camera_enable(5,TIME_STEP);
  wb_camera_enable(6,TIME_STEP);
  wb_camera_enable(7,TIME_STEP);
  wb_camera_enable(8,TIME_STEP);
  wb_camera_enable(9,TIME_STEP);
  wb_camera_enable(10,TIME_STEP);
  wb_camera_enable(11,TIME_STEP);
  // wb_gyro_enable(12,TIME_STEP);
  wb_light_sensor_enable(13,TIME_STEP);
  wb_receiver_enable(14,TIME_STEP);
  wb_range_finder_enable(15,TIME_STEP);
  wb_lidar_enable(16,1); //600
  
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  wb_lidar_enable_point_cloud(lidar);

  WbDeviceTag rec = wb_robot_get_device("receiver");

    
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
    robot_control(getTimer(timer));
    //go_forward();
    //stop();

    if (wb_receiver_get_queue_length(rec) > 0) 
    {
        const char *buffer = wb_receiver_get_data(rec);
        printf("Communicating: received \"%s\"\n", buffer);
    	 wb_receiver_next_packet(rec);
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
  }

  wb_robot_cleanup();

  return 0;
}
