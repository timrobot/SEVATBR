#ifndef measurements_h
#define measurements_h

#include <math.h>

// Note: all the following measurements are in cm and radians
// for length and angle respectively

// waist parameters
const static double waist_x[4] = { -6.4, 6.4, -6.4, 6.4 };
const static double waist_y[4] = { 27.3, 27.3, -27.3, -27.3 };
const static double waist_angle[4] = { 1.309, -0.2618, 2.798, -1.8326 };
const static double waist_pot_min[4] = { 0.0, M_PI_2, M_PI, 0.0 }; // not true values!
const static double waist_pot_max[4] = { M_PI_2, M_PI, M_PI + M_PI_2, -M_PI_2 };
const static int waist_pot_read_min[4] = { 19, 19, 19, 19 };
const static int waist_pot_read_max[4] = { 53, 53, 53, 53 };

// thigh parameters
const static double thigh_x = 3.1;
const static double thigh_z = 33.3;
const static double thigh_pivot_length = 6.4;
const static double thigh_pivot_angle = 2.3562;
const static double thigh_length = 27.3;
const static double thigh_upper_length = 7.0;
const static double thigh_lower_length = 20.3;

// shin parameters
const static double shin_length = 43.1;
const static double shin_upper_length = 33.0;
const static double shin_lower_length = 10.1;

// conversion parameters
const static double actuator_min = 33.1; // tentative...
const static double actuator_max = 58.0;
const static int actuator_read_min = 0;
const static int actuator_read_max = 100;
const static double pot_rad_ratio = 350.0;

#endif
