/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#ifndef ORANGE_AVOIDER_H
#define ORANGE_AVOIDER_H

#include <stdint.h>
#include <stdbool.h>

// settings
extern float oa_color_count_frac;

typedef struct obs_data_t {
    uint16_t x;
    uint16_t y;
    uint16_t width;
} obs_data_t;

typedef struct cv_test_global_t {
    uint8_t obstacle_num;
    bool updated;
    obs_data_t obs[10];
} cv_test_global;

extern float maxSpeed;
extern float max_heading_increment;

// functions
extern void orange_avoider_init(void);
extern void orange_avoider_periodic(void);


#endif

