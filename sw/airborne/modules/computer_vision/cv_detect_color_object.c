/*
 * Copyright (C) 2019 Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/cv_detect_object.h
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not
 */

// Own header
#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include "std.h"
#include "modules/orange_avoider/orange_avoider.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"

#define OBJECT_DETECTOR_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef COLOR_OBJECT_DETECTOR_FPS1
#define COLOR_OBJECT_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif
#ifndef COLOR_OBJECT_DETECTOR_FPS2
#define COLOR_OBJECT_DETECTOR_FPS2 0 ///< Default FPS (zero means run at camera fps)
#endif

// Filter Settings
uint8_t cod_lum_min1 = 0;
uint8_t cod_lum_max1 = 0;
uint8_t cod_cb_min1 = 0;
uint8_t cod_cb_max1 = 0;
uint8_t cod_cr_min1 = 0;
uint8_t cod_cr_max1 = 0;

uint8_t cod_lum_min2 = 0;
uint8_t cod_lum_max2 = 0;
uint8_t cod_cb_min2 = 0;
uint8_t cod_cb_max2 = 0;
uint8_t cod_cr_min2 = 0;
uint8_t cod_cr_max2 = 0;

bool cod_draw1 = false;
bool cod_draw2 = false;

// define global variables
struct color_object_t {
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  bool updated;
};
// struct color_object_t global_filters[2];

typedef struct {
    uint16_t x;
    uint16_t y;
} coordinates;

typedef struct {
    coordinates start;
    coordinates end;
} obs_pos;

cv_test_global global_filters_new[2];


// Function
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max,
                              int *idx, obs_pos *obstacle_coordinates);


void erosion(uint8_t **image, uint16_t rows, uint16_t cols, uint8_t kernel_size);
void dilation(uint8_t **image, uint16_t rows, uint16_t cols, uint8_t kernel_size);
void remove_mat(uint8_t **binary_image, uint32_t rows, uint32_t cols);
void remove_dirt(uint8_t **binary_image, uint32_t rows, uint32_t cols);
void find_obstacle(uint8_t **binary_image, uint16_t rows, uint16_t cols, obs_pos *obstacle_pos, int *num);
int compare_obs_pos(const void *a, const void *b);
int prune_obstacles(obs_pos *f_coord, obs_pos *obs_coord, int num_obs_coord);

/*
 * object_detector
 * @param img - input image to process
 * @param filter - which detection filter to process
 * @return img
 */
static struct image_t *object_detector(struct image_t *img, uint8_t filter)
{
  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;
  bool draw;

  switch (filter){
    case 1:
      lum_min = cod_lum_min1;
      lum_max = cod_lum_max1;
      cb_min = cod_cb_min1;
      cb_max = cod_cb_max1;
      cr_min = cod_cr_min1;
      cr_max = cod_cr_max1;
      draw = cod_draw1;
      break;
    case 2:
      lum_min = cod_lum_min2;
      lum_max = cod_lum_max2;
      cb_min = cod_cb_min2;
      cb_max = cod_cb_max2;
      cr_min = cod_cr_min2;
      cr_max = cod_cr_max2;
      draw = cod_draw2;
      break;
    default:
      return img;
  };

  int32_t x_c, y_c;

  obs_pos *obstacle_coordinates = (obs_pos *)malloc(2500 * sizeof(obs_pos));
  int idx = 0;

  // Filter and find centroid
  uint32_t count = find_object_centroid(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max, &idx, obstacle_coordinates);
  count = count > 10? 10: count;

  obs_pos *final_coordinates = &obstacle_coordinates[idx];

  // VERBOSE_PRINT("Color count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count, count_threshold, x_c, y_c);
  // VERBOSE_PRINT("centroid %d: (%d, %d) r: %4.2f a: %4.2f\n", camera, x_c, y_c,
  //       hypotf(x_c, y_c) / hypotf(img->w * 0.5, img->h * 0.5), RadOfDeg(atan2f(y_c, x_c)));

  // pthread_mutex_lock(&mutex);
  // global_filters[filter-1].color_count = count;
  // global_filters[filter-1].x_c = x_c;
  // global_filters[filter-1].y_c = y_c;
  // global_filters[filter-1].updated = true;
  // pthread_mutex_unlock(&mutex);

  pthread_mutex_lock(&mutex);
  for (uint32_t i = 0; i < count; i++) {
    global_filters_new[filter-1].obs[i].x = final_coordinates[i].start.x;
    global_filters_new[filter-1].obs[i].y = (final_coordinates[i].start.y + final_coordinates[i].end.y) / 2.0;
    global_filters_new[filter-1].obs[i].width = abs(final_coordinates[i].start.y + final_coordinates[i].end.y);
  }
  global_filters_new[filter-1].updated = true;
  global_filters_new[filter-1].obstacle_num = count;
  pthread_mutex_unlock(&mutex);

  // VERBOSE_PRINT("Final array new: %d\n", count);
  // for (size_t i = 0; i < count; i++) {
  //   VERBOSE_PRINT("(%d, %d) - (%d, %d)\n", final_coordinates[i].start.x, final_coordinates[i].start.y, final_coordinates[i].end.x, final_coordinates[i].end.y);
  // }

  free(obstacle_coordinates);

  return img;
}

struct image_t *object_detector1(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector1(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 1);
}

struct image_t *object_detector2(struct image_t *img, uint8_t camera_id);
struct image_t *object_detector2(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  return object_detector(img, 2);
}

void color_object_detector_init(void)
{
  // memset(global_filters, 0, 2*sizeof(struct color_object_t));
  memset(global_filters_new, 0, 2*sizeof(cv_test_global));
  pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA1
  #ifdef COLOR_OBJECT_DETECTOR_LUM_MIN1
    cod_lum_min1 = COLOR_OBJECT_DETECTOR_LUM_MIN1;
    cod_lum_max1 = COLOR_OBJECT_DETECTOR_LUM_MAX1;
    cod_cb_min1 = COLOR_OBJECT_DETECTOR_CB_MIN1;
    cod_cb_max1 = COLOR_OBJECT_DETECTOR_CB_MAX1;
    cod_cr_min1 = COLOR_OBJECT_DETECTOR_CR_MIN1;
    cod_cr_max1 = COLOR_OBJECT_DETECTOR_CR_MAX1;
  #endif
  #ifdef COLOR_OBJECT_DETECTOR_DRAW1
    cod_draw1 = COLOR_OBJECT_DETECTOR_DRAW1;
  #endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, object_detector1, COLOR_OBJECT_DETECTOR_FPS1, 0);
#endif

#ifdef COLOR_OBJECT_DETECTOR_CAMERA2
  #ifdef COLOR_OBJECT_DETECTOR_LUM_MIN2
    cod_lum_min2 = COLOR_OBJECT_DETECTOR_LUM_MIN2;
    cod_lum_max2 = COLOR_OBJECT_DETECTOR_LUM_MAX2;
    cod_cb_min2 = COLOR_OBJECT_DETECTOR_CB_MIN2;
    cod_cb_max2 = COLOR_OBJECT_DETECTOR_CB_MAX2;
    cod_cr_min2 = COLOR_OBJECT_DETECTOR_CR_MIN2;
    cod_cr_max2 = COLOR_OBJECT_DETECTOR_CR_MAX2;
  #endif
  #ifdef COLOR_OBJECT_DETECTOR_DRAW2
    cod_draw2 = COLOR_OBJECT_DETECTOR_DRAW2;
  #endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA2, object_detector2, COLOR_OBJECT_DETECTOR_FPS2, 1);
#endif
}

/*
 * find_object_centroid
 *
 * Finds the centroid of pixels in an image within filter bounds.
 * Also returns the amount of pixels that satisfy these filter bounds.
 *
 * @param img - input image to process formatted as YUV422.
 * @param p_xc - x coordinate of the centroid of color object
 * @param p_yc - y coordinate of the centroid of color object
 * @param lum_min - minimum y value for the filter in YCbCr colorspace
 * @param lum_max - maximum y value for the filter in YCbCr colorspace
 * @param cb_min - minimum cb value for the filter in YCbCr colorspace
 * @param cb_max - maximum cb value for the filter in YCbCr colorspace
 * @param cr_min - minimum cr value for the filter in YCbCr colorspace
 * @param cr_max - maximum cr value for the filter in YCbCr colorspace
 * @param draw - whether or not to draw on image
 * @return number of pixels of image within the filter bounds.
 */
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max,
                              int *idx, obs_pos *obstacle_coordinates)
{
  uint32_t cnt = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;
  const float crop_factor = 0.5;

  int w_scan_limit = img->w * crop_factor;

  int num_obs_coord = 0;

  int num_final_obs = 0;

  uint8_t** Filtered_img = (uint8_t**)malloc(img->h * sizeof(uint8_t*));
  for (int i = 0; i < img->h; i++) {
    Filtered_img[i] = (uint8_t*)malloc(img->w * sizeof(uint8_t));
    memset(Filtered_img[i], 0, img->w * sizeof(uint8_t));
  }

  // Go through all the pixels in y direction but only half in x direction
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < w_scan_limit; x ++) {
      // Check if the color is inside the specified values
      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x];      // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
        //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
      } else {
        // Uneven x
        up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
        //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
        vp = &buffer[y * 2 * img->w + 2 * x];      // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }


      if ( (*yp >= lum_min) && (*yp <= lum_max) &&
          (*up >= cb_min ) && (*up <= cb_max ) &&
          (*vp >= cr_min ) && (*vp <= cr_max )) {
        Filtered_img[y][x] = 1;
      } else {
        Filtered_img[y][x] = 0;
      }

    }
  }

  // Can make kernel for erosion to 4 if any issues
  // Processing only part of the image
  erosion(Filtered_img, img->h, w_scan_limit, 4);
  dilation(Filtered_img, img->h, w_scan_limit, 6);
  remove_dirt(Filtered_img, img->h, w_scan_limit);
  remove_mat(Filtered_img, img->h, w_scan_limit);

  // Draw red pixels wherever we detect floor
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x ++) {
      // Check if the color is inside the specified values
      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x];      // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
        //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
      } else {
        // Uneven x
        up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
        //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
        vp = &buffer[y * 2 * img->w + 2 * x];      // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }

      if (Filtered_img[y][x] == 1) {
          *yp = 81;
          *up = 90;
          *vp = 240;

      }
    }
  }

  // obs_pos *obstacle_coordinates = (obs_pos *)malloc(2500 * sizeof(obs_pos));
  find_obstacle(Filtered_img, img->h, img->w, obstacle_coordinates, &num_obs_coord);

  // Sort the array
  qsort(obstacle_coordinates, num_obs_coord, sizeof(obs_pos), compare_obs_pos);

  obs_pos *final_coordinates = &obstacle_coordinates[num_obs_coord];
  *idx = num_obs_coord;

  num_final_obs = prune_obstacles(final_coordinates, obstacle_coordinates, num_obs_coord);

  // int prev_point_y = -1, y_idx = -1, x_idx = -1;

  // for (int i = 0; i < num_obs_coord; i++) {

  //     if ((prev_point_y == -1) || (abs(prev_point_y - obstacle_coordinates[i].start.y) >= 10)) {

  //       if (prev_point_y != -1) {

  //         final_coordinates[num_final_obs].start.x = obstacle_coordinates[x_idx].start.x;
  //         final_coordinates[num_final_obs].start.y = obstacle_coordinates[y_idx].start.y;

  //         final_coordinates[num_final_obs].end.x = obstacle_coordinates[y_idx].end.x;
  //         final_coordinates[num_final_obs].end.y = obstacle_coordinates[y_idx].end.y;

  //         num_final_obs++;

  //       }

  //       y_idx = i;
  //       x_idx = i;


  //     } else {
        
  //       if (obstacle_coordinates[i].start.x < obstacle_coordinates[y_idx].start.x) {
  //         x_idx = i;
  //       }

  //     }

  //     prev_point_y = obstacle_coordinates[i].start.y;
  // }

  // // Add the last obstacle
  // if (num_obs_coord > 0) {
  //   final_coordinates[num_final_obs].start.x = obstacle_coordinates[x_idx].start.x;
  //   final_coordinates[num_final_obs].start.y = obstacle_coordinates[y_idx].start.y;

  //   final_coordinates[num_final_obs].end.x = obstacle_coordinates[y_idx].end.x;
  //   final_coordinates[num_final_obs].end.y = obstacle_coordinates[y_idx].end.y;

  //   num_final_obs++;
  // }



  // Draw the start line of obstacle
  for (int i = 0; i < num_final_obs; i++) {
    int x = 0, y = 0;

    x = final_coordinates[i].start.x;
    y = final_coordinates[i].start.y;

    for (int j = x; j < img->w; j++) {

        uint8_t *yp, *up, *vp;
        if (j % 2 == 0) {
          up = &buffer[y * 2 * img->w + 2 * j];      // U
          yp = &buffer[y * 2 * img->w + 2 * j + 1];  // Y1
          vp = &buffer[y * 2 * img->w + 2 * j + 2];  // V
        } else {
          up = &buffer[y * 2 * img->w + 2 * j - 2];  // U
          vp = &buffer[y * 2 * img->w + 2 * j];      // V
          yp = &buffer[y * 2 * img->w + 2 * j + 1];  // Y2
        }

          *yp = 210;
          *up = 16;
          *vp = 146;
    }

  }

  // Draw the end line of obstacle
  for (int i = 0; i < num_final_obs; i++) {
    int x = 0, y =0;

    x = final_coordinates[i].end.x;
    y = final_coordinates[i].end.y;

    for (int j = x; j < img->w; j++) {

        uint8_t *yp, *up, *vp;
        if (j % 2 == 0) {
          up = &buffer[y * 2 * img->w + 2 * j];      // U
          yp = &buffer[y * 2 * img->w + 2 * j + 1];  // Y1
          vp = &buffer[y * 2 * img->w + 2 * j + 2];  // V
        } else {
          up = &buffer[y * 2 * img->w + 2 * j - 2];  // U
          vp = &buffer[y * 2 * img->w + 2 * j];      // V
          yp = &buffer[y * 2 * img->w + 2 * j + 1];  // Y2
        }

        *yp = 106;
        *up = 202;
        *vp = 222;
    }

  }

  // // Go through all the pixels
  // for (uint16_t y = 0; y < img->h; y++) {
  //   for (uint16_t x = 0; x < img->w; x ++) {
  //     // Check if the color is inside the specified values
  //     uint8_t *yp, *up, *vp;
  //     if (x % 2 == 0) {
  //       // Even x
  //       up = &buffer[y * 2 * img->w + 2 * x];      // U
  //       yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
  //       vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
  //       //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
  //     } else {
  //       // Uneven x
  //       up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
  //       //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
  //       vp = &buffer[y * 2 * img->w + 2 * x];      // V
  //       yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
  //     }

  //     if (Filtered_img[y][x] == 1) {
  //         *yp = 255;  // make pixel brighter in image
  //     }
  //   }
  // }

  // char filename[100];
  // snprintf(filename, sizeof(filename), "/home/hardik/Documents/TU_Delft/Q3/MAV/binary_image_%d.bmp", img_num);
  // save_binary_image(Filtered_img, img->w, img->h, filename);
  // img_num++;

  if (cnt > 0) {
    *p_xc = (int32_t)roundf(tot_x / ((float) cnt) - img->w * 0.5f);
    *p_yc = (int32_t)roundf(img->h * 0.5f - tot_y / ((float) cnt));
  } else {
    *p_xc = 0;
    *p_yc = 0;
  }

      // Free memory allocated for the image
    for (int i = 0; i < img->h; i++) {
        free(Filtered_img[i]);
    }
    free(Filtered_img);
    // free(obstacle_coordinates);

  return num_final_obs;
}

void color_object_detector_periodic(void)
{
  // static struct color_object_t local_filters[2];
  static cv_test_global local_filters[2];
  static uint64_t seq1 = 0, seq2 = 0;

  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters_new, 2*sizeof(cv_test_global));
  pthread_mutex_unlock(&mutex);

  if(local_filters[0].updated){
    // AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
    //     0, 0, local_filters[0].color_count, 0);

    seq1++;
    AbiSendMsgPAYLOAD_DATA(COLOR_OBJECT_DETECTION1_ID, 0xFF, 0, sizeof(cv_test_global), (uint8_t*)&local_filters[0]);

    // for (int i = 0; i < local_filters[0].obstacle_num; i++) {
    //   VERBOSE_PRINT("S: x: %d, y: %d, width: %d\n", local_filters[0].obs[i].x, local_filters[0].obs[i].y, local_filters[0].obs[i].width);
    // }

    
    local_filters[0].updated = false;
  }
  if(local_filters[1].updated){
    // AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, local_filters[1].x_c, local_filters[1].y_c,
    //     0, 0, local_filters[1].color_count, 1);

    seq2++;
    AbiSendMsgPAYLOAD_DATA(COLOR_OBJECT_DETECTION2_ID, 0xFF, 0, sizeof(cv_test_global), (uint8_t*)&local_filters[1]);

    local_filters[1].updated = false;
  }
}

void erosion(uint8_t **image, uint16_t rows, uint16_t cols, uint8_t kernel_size) {
    uint16_t i, j;
    int m, n, min_val;

    // Allocate memory for the temporary image
    int **temp_image = (int **)malloc(rows * sizeof(int *));
    for (i = 0; i < rows; i++) {
        temp_image[i] = (int *)malloc(cols * sizeof(int));
    }

    // Iterate through each pixel in the image
    for (i = 0; i < rows; i++) {
        for (j = 0; j < cols; j++) {
            min_val = 255; // Initialize min_val to maximum intensity value

            // Iterate through the kernel
            for (m = -kernel_size / 2; m <= kernel_size / 2; m++) {
                for (n = -kernel_size / 2; n <= kernel_size / 2; n++) {
                    // Check if the current pixel is within the bounds of the image
                    if (i + m >= 0 && i + m < rows && j + n >= 0 && j + n < cols) {
                        // Update min_val with the minimum pixel value within the kernel
                        if (image[i + m][j + n] < min_val) {
                            min_val = image[i + m][j + n];
                        }
                    }
                }
            }

            // Assign the minimum pixel value to the corresponding pixel in the temporary image
            temp_image[i][j] = min_val;
        }
    }

    // Copy the result from the temporary image back to the original image
    for (i = 0; i < rows; i++) {
        for (j = 0; j < cols; j++) {
            image[i][j] = temp_image[i][j];
        }
    }

    // Free memory allocated for the temporary image
    for (i = 0; i < rows; i++) {
        free(temp_image[i]);
    }
    free(temp_image);
}


void dilation(uint8_t **image, uint16_t rows, uint16_t cols, uint8_t kernel_size) {
    uint16_t i, j;
    int m, n, max_val;

    // Allocate memory for the temporary image
    int **temp_image = (int **)malloc(rows * sizeof(int *));
    for (i = 0; i < rows; i++) {
        temp_image[i] = (int *)malloc(cols * sizeof(int));
    }

    // Iterate through each pixel in the image
    for (i = 0; i < rows; i++) {
        for (j = 0; j < cols; j++) {
            max_val = 0; // Initialize max_val to minimum intensity value

            // Iterate through the kernel
            for (m = -kernel_size / 2; m <= kernel_size / 2; m++) {
                for (n = -kernel_size / 2; n <= kernel_size / 2; n++) {
                    // Check if the current pixel is within the bounds of the image
                    if (i + m >= 0 && i + m < rows && j + n >= 0 && j + n < cols) {
                        // Update max_val with the maximum pixel value within the kernel
                        if (image[i + m][j + n] > max_val) {
                            max_val = image[i + m][j + n];
                        }
                    }
                }
            }

            // Assign the maximum pixel value to the corresponding pixel in the temporary image
            temp_image[i][j] = max_val;
        }
    }

    // Copy the result from the temporary image back to the original image
    for (i = 0; i < rows; i++) {
        for (j = 0; j < cols; j++) {
            image[i][j] = temp_image[i][j];
        }
    }

    // Free memory allocated for the temporary image
    for (i = 0; i < rows; i++) {
        free(temp_image[i]);
    }
    free(temp_image);
}



// Function to remove the mat from the binary image
void remove_mat(uint8_t **binary_image, uint32_t rows, uint32_t cols) {
    uint8_t prev_pixel = 0, pixel_value = 0;
    int on_mat = 0;
    coordinates mat_start = {.x = 0, .y = 0};
    coordinates mat_end = {.x = 0, .y = 0};

    for (int row = rows - 1; row >= 0; row--) {
        prev_pixel = 0;
        for (int col = cols - 1; col >= 0; col--) {

            pixel_value = binary_image[row][col];

            if (pixel_value == 0 && prev_pixel == 1 && on_mat == 0) {
                mat_end.x = col; mat_end.y = row;
                on_mat = 1;
            } else if (on_mat == 1 && (pixel_value == 1 || col == 0)) {
                mat_start.x = col; mat_start.y = row;
                on_mat = 0;

                // Set the mat region to 1s
                for (int c = mat_start.x; c <= mat_end.x; c++) {
                    binary_image[row][c] = 1;
                }
            }

            prev_pixel = pixel_value;
        }
    }
}

// Function to remove the mat from the binary image
void remove_dirt(uint8_t **binary_image, uint32_t rows, uint32_t cols) {
    uint8_t prev_pixel = 0, pixel_value = 0;
    coordinates dirt_start = {.x = 0, .y = 0};
    coordinates dirt_end = {.x = 0, .y = 0};
    int on_dirt = 0;
    uint32_t dist = 0;

    for (uint16_t col = 10; col < cols; col++) {
        prev_pixel = 0;
        for (uint16_t row = 0; row < rows; row++) {

            pixel_value = binary_image[row][col];

            if (pixel_value == 1 && prev_pixel == 0 && on_dirt == 0) {
                dirt_start.x = col; dirt_start.y = row;
                on_dirt = 1;
            } else if (on_dirt == 1 && (pixel_value == 0 || row == rows)) {
                dirt_end.x = col; dirt_end.y = row;
                on_dirt = 0;

                dist = ((dirt_end.x - dirt_start.x)^2) + ((dirt_end.y - dirt_start.y)^2);

                if ((dist >=1) && (dist < 15)) {
                  // Set the white dirt to 0
                  for (int r = dirt_start.y; r <= dirt_end.y; r++) {
                      binary_image[r][col] = 0;
                  }
                }
            }

            prev_pixel = pixel_value;
        }
    }
}


void find_obstacle(uint8_t **binary_image, uint16_t rows, uint16_t cols, obs_pos *obstacle_pos, int *num) {
  
  coordinates obstacle_start = {.x = 0, .y = 0};
  coordinates obstacle_end = {.x = 0, .y = 0};
  uint32_t dist = 0;
  const int MAX_DIST_SQ = 15^2;
  const int CN_MAX_DIST_SQ = 200^2;
  int num_obstacles = 0;

  int inside_obstacle = 0, corner_obstacle = 0;
  int prev_pixel = 0, pixel_value = 0;

  for (uint16_t col = 0; col < cols; col++) {
      // Flag to indicate if we are currently inside an obstacle
      inside_obstacle = 0;
      prev_pixel = -1;

      // Iterate through each row in the column
      for (uint16_t row = 0; row < rows; row++) {
          // Get the pixel value
          pixel_value = binary_image[row][col];

          // It can't differentiate between obstacle and no green floor anymore
          // // Detects obstacle on left edge
          // TODO: Maybe try to add larger width to qualify these edges as obstacles
          // if (row <= 5 && col <= 5 && pixel_value == 0) {
          //   prev_pixel = 1;
          //   corner_obstacle = 1;
          // }

          // // Detects obstacle on right edge
          // if (row >= (rows - 5) && col <= 5 && inside_obstacle == 1) {
          //   pixel_value = 1;
          //   corner_obstacle = 1;
          // }

          // Check for transition from white to black (start of obstacle)
          if (inside_obstacle == 0 && pixel_value == 0 && prev_pixel == 1) {
              obstacle_start.x = col; obstacle_start.y = row;
              inside_obstacle = 1;
          }
          // Check for transition from black to white (end of obstacle)
          else if (inside_obstacle == 1 && pixel_value == 1 && prev_pixel == 0) {
            obstacle_end.x = col; obstacle_end.y = row;

            dist = ((obstacle_end.x - obstacle_start.x)^2) + ((obstacle_end.y - obstacle_start.y)^2);

            if (dist > MAX_DIST_SQ) {

                // if ((corner_obstacle == 0) || ((corner_obstacle == 1) && (dist > CN_MAX_DIST_SQ))) {
                obstacle_pos[num_obstacles].start.x = obstacle_start.x;
                obstacle_pos[num_obstacles].start.y = obstacle_start.y;
                obstacle_pos[num_obstacles].end.x = obstacle_end.x;
                obstacle_pos[num_obstacles].end.y = obstacle_end.y;
                num_obstacles++;
                // }
                
            }
              inside_obstacle = 0;
              corner_obstacle = 0;
          }

          
          prev_pixel = pixel_value;
      }
    }

    *num = num_obstacles;
}


// Custom comparison function for sorting obs_pos array by start.y and then start.x
int compare_obs_pos(const void *a, const void *b) {
    const obs_pos *obs_a = (const obs_pos *)a;
    const obs_pos *obs_b = (const obs_pos *)b;
 
    // First, compare by start.y
    if (obs_a->start.y != obs_b->start.y) {
        return obs_a->start.y - obs_b->start.y;
    }
 
    // If start.y is equal, compare by start.x
    return obs_a->start.x - obs_b->start.x;
}

int prune_obstacles(obs_pos *f_coord, obs_pos *obs_coord, int num_obs_coord) {
  int num_final_obs = 0;
  coordinates obstacle_start = {.x = 0, .y = 0};
  coordinates obstacle_end = {.x = 0, .y = 0};
  const int MAX_DIST = 40;
  int prev_point_sy = -1, prev_point_ey = -1, y_idx = -1, x_idx = -1;
  int length = 0;

  for (int i = 0; i < num_obs_coord; i++) {

      if ((prev_point_sy == -1) || ((abs(prev_point_sy - obs_coord[i].start.y) >= 10) && (abs(prev_point_ey - obs_coord[i].end.y) >= 10))) {

        if (prev_point_sy != -1) {

          obstacle_start.x = obs_coord[x_idx].start.x;
          obstacle_start.y = obs_coord[y_idx].start.y;

          obstacle_end.x = obs_coord[y_idx].end.x;
          obstacle_end.y = obs_coord[y_idx].end.y;

          if ((abs(obstacle_end.y - obstacle_start.y) > MAX_DIST) && (length > 5)) {
            f_coord[num_final_obs].start.x = obs_coord[x_idx].start.x;
            f_coord[num_final_obs].start.y = obs_coord[y_idx].start.y;

            f_coord[num_final_obs].end.x = obs_coord[y_idx].end.x;
            f_coord[num_final_obs].end.y = obs_coord[y_idx].end.y;

            num_final_obs++;
          }
        }

        y_idx = i;
        x_idx = i;
        length = 0;

      }
      else {
  
        if (obs_coord[i].start.x < obs_coord[y_idx].start.x) {
          x_idx = i;
        }

        length++;

      }

      prev_point_sy = obs_coord[i].start.y;
      prev_point_ey = obs_coord[i].end.y;
  }

  // Add the last obstacle
  if (num_obs_coord > 0) {
    obstacle_start.x = obs_coord[x_idx].start.x;
    obstacle_start.y = obs_coord[y_idx].start.y;

    obstacle_end.x = obs_coord[y_idx].end.x;
    obstacle_end.y = obs_coord[y_idx].end.y;

    if (abs(obstacle_end.y - obstacle_start.y) > MAX_DIST) {
      f_coord[num_final_obs].start.x = obs_coord[x_idx].start.x;
      f_coord[num_final_obs].start.y = obs_coord[y_idx].start.y;

      f_coord[num_final_obs].end.x = obs_coord[y_idx].end.x;
      f_coord[num_final_obs].end.y = obs_coord[y_idx].end.y;
      num_final_obs++;
    }
  }

  return num_final_obs;
}