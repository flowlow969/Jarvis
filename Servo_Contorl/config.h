#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

#include "servo.h"

// Beschreiben der Konfigurationen der Servos

Servo  servo_A = {
  .servo_ID = 0,
  .set_angel = 0,
  .min_angel = -105,
  .max_angel = 105,
  .map_min = 0,
  .map_max = 450
};

Servo  servo_B = {
  .servo_ID = 1,
  .set_angel = 90,
  .min_angel = -15,
  .max_angel = 110,
  .map_min = 300,
  .map_max = 10
};

Servo  servo_C = {
  .servo_ID = 2,
  .set_angel = 0,
  .min_angel = -10,
  .max_angel = 105,
  .map_min = 15,
  .map_max = 305
};

Servo  servo_D = {
  .servo_ID = 3,
  .set_angel = 130,
  .min_angel = 130,
  .max_angel = -10,
  .map_min = 0,
  .map_max = 300
};

Arm  Jarvis = {
  .servo_0 = servo_A,
  .servo_1 = servo_B,
  .servo_2 = servo_C,
  .servo_3 = servo_D,  
  .length_0 = 63.400,
  .length_1 = 151.00,
  .length_2 = 161.00
};

// Initale Position festlegen
Positsion Postsions = {
  .target_X = 150.00,
  .target_Y = 0.00,
  .target_Z = 200.00
};

#endif