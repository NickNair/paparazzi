/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the navigation mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 */

#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE


#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#define clip(x, min, max)\
  ({ __typeof__ (x) _x = (x); \
     __typeof__ (min) _min = (min); \
     __typeof__ (max) _max = (max); \
     _x > _min ? (_x < _max ? _x : _max) : _min; })

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t moveWaypointForwardWithOffsetAngle(uint8_t waypoint, float distanceMeters, float offsetAngle);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t calculateForwardsWithOffsetAngle(struct EnuCoor_i *new_coor, float distanceMeters, float offsetAngle);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(int min_heading_num);
static uint8_t chooseRandomIncrementAvoidance(void);
static uint8_t chooseIncrementAvoidance(void);
static int max_width(cv_test_global obs_info, int* idx_arr, int size);
float euler_angle;
enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  CRITICAL_ZONE,
  OUT_OF_BOUNDS
};

cv_test_global cv_test;

uint8_t flag_critical = 0;
// define settings
float oa_color_count_frac = 0.22f; // Threshold for allowed obstacle pixels in the frame
uint32_t color_count_threshold; // To be initialized in init()
float oa_mid_pix_count_frac = 0.04f; // Threshold for allowed obstacle pixels in the middle of the frame
//uint32_t mid_pix_count_threshold; // To be initialized in init()

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
uint32_t color_count = 0;                // orange color count from color filter for obstacle detection
uint32_t cnt_L = 0;
uint32_t cnt_M = 0;
uint32_t cnt_R = 0;
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
float max_heading_increment = 20.0f;
float min_heading_increment = 5.0f;    // Min heading indcrement [deg]
float heading_increment = 20.f;          // Current setting for heading angle increment [deg]
int heading_num = 0;
int lockChangeHeading = 0;              // If the drone is in safe mode and changing its heading to remove obstacles from its middle, don't do this infinitely
float maxSpeed = 1.0;               // max waypoint displacement [m]
int obs_width_threshold = 80;

const int16_t max_trajectory_confidence = 6; // number of consecutive negative object detections to be sure we are obstacle free


/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event color_detection_ev;
// static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
//                                int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
//                                int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
//                                int32_t quality, int16_t __attribute__((unused)) extra)
// {
//   color_count = quality;
// }
//
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               uint32_t stamp, int32_t __attribute__((unused)) data_type,
                               uint32_t size, uint8_t *buf)
{
  // if(stamp != 101 || size != 16) return; // Check if we actually receive the message of color detection

  memcpy(&cv_test, (cv_test_global*) buf, size);

  //VERBOSE_PRINT("NUM OBS: %d\n", cv_test.obstacle_num);

}

// Returns the largest width of all obstacles in the selected part of the screen in pixels
int max_width(cv_test_global obs_info, int *idx_arr, int size)
{
    int width = obs_info.obs[idx_arr[0]].width;
    int _max = width;

    for (int i = 0; i < size; i++) {
        width = obs_info.obs[idx_arr[i]].width;
        if (width > _max) {
            _max = width;
        }
    }
    return _max;
}

int max_width_all(cv_test_global obs_info)
{
  if(obs_info.obstacle_num == 0) return 0;
  int width = obs_info.obs[0].width;
  int _max = width;

  for (int i = 0; i < obs_info.obstacle_num; i++) {
      width = obs_info.obs[i].width;
      if (width > _max) {
          _max = width;
      }
  }
  return _max;
}

/*
 * Initialisation function, setting the colour filter, random seed and heading_increment
 */
void orange_avoider_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgPAYLOAD_DATA(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void orange_avoider_periodic(void)
{
  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }

  cnt_L = 0;
  cnt_M = 0;
  cnt_R = 0;
  int index_L[10] = {0}, index_M[10] = {0}, index_R[10] = {0}; 
  for (int i = 0; i < cv_test.obstacle_num; i ++) {
    if(cv_test.obs[i].y <= 170){
      index_L[cnt_L] = i;
      cnt_L++;
    } else if (cv_test.obs[i].y <= 350){
      index_M[cnt_M] = i;
      cnt_M++;
    } else {
      index_R[cnt_R] = i;
      cnt_R++;
    }
  }
  VERBOSE_PRINT("L: %d M: %d R: %d\n", cnt_L, cnt_M, cnt_R);
  

  color_count_threshold = cv_test.obstacle_num;
  //mid_pix_count_threshold = 0;

    // update our safe confidence using color threshold
  //if(color_count < color_count_threshold){ 
  if (navigation_state != CRITICAL_ZONE) {
      if(max_width_all(cv_test) < obs_width_threshold){ 
        obstacle_free_confidence++;
      }
      else { // Remove confidence if there are too many obstacles in the field
        obstacle_free_confidence = 0; 

    VERBOSE_PRINT("DETECTING OBSTACLE! (width = %d) Setting confidence level to %d\n", max_width_all(cv_test), obstacle_free_confidence);
      }
  } else {
        obstacle_free_confidence = 0;
  }
  // Bound the speed depending on the amount of obstacles in sight
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);
  //float moveDistance = clip(1 - 0.5*color_count/color_count_threshold, 0, 1) * obstacle_free_confidence * maxDistance / max_trajectory_confidence;
  float moveDistance = maxSpeed * obstacle_free_confidence / (float)max_trajectory_confidence;

  uint32_t _green = cv_test.green_count;

  /*if ((!_green) && navigation_state != SEARCH_FOR_SAFE_HEADING) {
            navigation_state = 
            uint8_t flag_critical = 0;CRITICAL_ZONE; 
            color_count_threshold = 1;
            VERBOSE_PRINT("Confidence: %d\n", obstacle_free_confidence);
        VERBOSE_PRINT("CASE GEEN GREEN \n");
    }*/

  switch (navigation_state){
    case SAFE:
      if(cnt_M > 0 ) {// && max_width(cv_test, index_M, cnt_M) <= obs_width_threshold){ // If there is an obstacle in the middle, but still far, move aside and continue flying
        // Move waypoint slightly sideways
        if(lockChangeHeading == 0) { // Only change heading once to avoid oscillations
          chooseIncrementAvoidance(); 
          lockChangeHeading = 1;
        }

        increase_nav_heading(0);
        moveWaypointForwardWithOffsetAngle(WP_TRAJECTORY, 1.5f * moveDistance, (heading_increment / fabs(heading_increment)) * 45.0); // Moves waypoint sideways as well to start avoidance motion early, This is reset once the obstacle is out of view
      }
      /*else {
        // Move waypoint forward
        moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
        lockChangeHeading = 0; // Reset heading lock
      }*/

      // Make sure GOAL is inside bounds
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence == 0){
        navigation_state = OBSTACLE_FOUND;
      } else {
        moveWaypointForward(WP_GOAL, moveDistance);
      }


      if (_green < 8000) {
            navigation_state = CRITICAL_ZONE; 
            color_count_threshold = 1;
            flag_critical = 0;
      }
        
      break;
    case OBSTACLE_FOUND:
      // stop
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);

      // randomly select new search direction
      chooseIncrementAvoidance();
      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;
    case SEARCH_FOR_SAFE_HEADING:


      increase_nav_heading(0);
      // make sure we have a couple of good readings before declaring the way safe
      if (obstacle_free_confidence >= 3){
        lockChangeHeading = 0;
        navigation_state = SAFE;
      }
      break;

    case CRITICAL_ZONE: 
      //Stop
      //chooseIncrementAvoidance();
      if (!flag_critical) {
          waypoint_move_here_2d(WP_GOAL);
          waypoint_move_here_2d(WP_TRAJECTORY);
          flag_critical = 1;
       }
      heading_increment = 5;
      increase_nav_heading(0);
      moveWaypointForward(WP_TRAJECTORY, 0.0);
      moveWaypointForward(WP_GOAL, 0.0);

      if (_green < 12000) {
             VERBOSE_PRINT("CONDITION = %d, GREEN = %d\n", navigation_state, _green);
             navigation_state = CRITICAL_ZONE;
      }
      else
         navigation_state = SAFE;
    break;

    case OUT_OF_BOUNDS:
      // Change increment direction in a 'smart' way
      if(lockChangeHeading == 0) { // Only change heading once to avoid oscillations
        chooseIncrementAvoidance(); 
        lockChangeHeading = 1;
      }

      increase_nav_heading(2);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        // add offset to head back into arena
        increase_nav_heading(0);

        // Reset heading lock
        lockChangeHeading = 0;

        // reset safe counter
        obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      }
      break;
    default:
      break;
  }
  return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(int min_heading_num)
{
  if (heading_num > min_heading_num) { // if this is not the first try, reduce rotation to 5 degrees, but keep same direction
      if (fabs(heading_increment) > min_heading_increment) {
          heading_increment = heading_increment/4.0;
      } 
      heading_increment = (heading_increment < min_heading_increment) ? min_heading_increment : heading_increment;
  }
  heading_num++;
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(heading_increment);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  nav.heading = new_heading;

  VERBOSE_PRINT("Increasing heading (%i) with %f degrees to %f\n", heading_num, heading_increment, DegOfRad(new_heading));
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Calculates coordinates of distance forward with some offset in degrees and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForwardWithOffsetAngle(uint8_t waypoint, float distanceMeters, float offsetAngle)
{
  struct EnuCoor_i new_coor;
  calculateForwardsWithOffsetAngle(&new_coor, distanceMeters, offsetAngle);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  //VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
                // POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                // stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}


/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading with an additional offset in degrees
 */
uint8_t calculateForwardsWithOffsetAngle(struct EnuCoor_i *new_coor, float distanceMeters, float offsetAngle)
{
  float heading  = stateGetNedToBodyEulers_f()->psi + RadOfDeg(offsetAngle);

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  //VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                // POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  heading_num = 0;
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_increment = max_heading_increment;
    VERBOSE_PRINT("Set avoidance Random increment to: %f\n", heading_increment);
  } else {
    heading_increment = -max_heading_increment;
    VERBOSE_PRINT("Set avoidance Random increment to: %f\n", heading_increment);
  }

    // Check if the new direction points to a space within the cyberzone bounds, if not, take the other direction
  moveWaypointForwardWithOffsetAngle(WP_TRAJECTORY, 1.5f, heading_increment / fabs(heading_increment) * 90.0f); // checks x degrees left or right depending on current heading angle
  if(InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY)) == 0){
    heading_increment = -(heading_increment); // If the new direction points outside the cyberzoo, just take the other direction
    VERBOSE_PRINT("Flipped sign of avoidance increment because it is near the edge of the cyberzoo.\n");
  }

  return false;
}

/*
 * Sets the variable 'heading_increment' positive/negative depending on the amount of obstacles left/right on the screen
 */
uint8_t chooseIncrementAvoidance(void)
{
  heading_num = 0;
  if(cnt_L == cnt_R && (cnt_M != 0)){ // If equal obstacle counts, move to a random direction
    //chooseRandomIncrementAvoidance();
    int heading_increment_sign = (heading_increment > 0) ? 1: -1;
    heading_increment = heading_increment_sign * max_heading_increment;
  }
  else if(cnt_L > cnt_R){ // If more obstacles left than right, move clockwise
    heading_increment = max_heading_increment;
  }
  else{ // If more obstacles right than left, move counter clockwise
    heading_increment = -max_heading_increment;  //make it negative again (TEST)
  }

  // Check if the new direction points to a space within the cyberzone bounds, if not, take the other direction
  moveWaypointForwardWithOffsetAngle(WP_TRAJECTORY, 1.5f, heading_increment / fabs(heading_increment) * 90.0f); // checks x degrees left or right depending on current heading angle
  if(InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY)) == 0){
    heading_increment = -(heading_increment); // If the new direction points outside the cyberzoo, just take the other direction
    VERBOSE_PRINT("Flipped sign of avoidance increment because it is near the edge of the cyberzoo.\n");
  }

  VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);

  return false;
}
