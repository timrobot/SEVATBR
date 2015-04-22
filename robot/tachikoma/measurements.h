#ifndef measurements_h
#define measurements_h

// Note: all the following measurements are in cm and radians
// for length and angle respectively

// waist parameters
const static double waist_x[4] = { -6.4, 6.4, -6.4, 6.4 };
const static double waist_y[4] = { 27.3, 27.3, -27.3, -27.3 };
const static double waist_angle[4] = { 1.309, -0.2618, 2.798, -1.8326 };
const static double waist_pot_min[4] = { 0.0, M_PI_2, M_PI, 0.0 }; // not true values!
const static double waist_pot_max[4] = { M_PI_2, M_PI, M_PI + M_PI_2, -M_PI_2 };
const static double waist_pot_reading_min[4] = { 19.0, 19.0, 19.0, 19.0 };
const static double waist_pot_reading_max[4] = { 53.0, 53.0, 53.0, 53.0 };

// thigh parameters
const static double thigh_x = 3.1;
const static double thigh_z = 33.3;
const static double thigh_pivot_length = 6.4;
const static double thigh_pivot_angle = 2.3562;
const static double thigh_length = 27.3;
const static double thigh_upper_length = 7.0;
const static double thigh_lower_length = 20.3;
const static double thigh_actuator_min = 33.1; // tentative...
const static double thigh_actuator_max = 58.0;

// shin parameters
const static double shin_length = 43.1;
const static double shin_actuator_min = 29.5;
const static double shin_actuator_max = 50.0;
const static double shin_upper_length = 33.0;
const static double shin_lower_length = 10.1;

#endif
