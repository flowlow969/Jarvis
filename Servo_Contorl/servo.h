#ifndef SERVO_H
#define SERVO_H



typedef struct {
  int servo_ID;
  float set_angel;
  float min_angel;
  float max_angel;
  int map_min;
  int map_max;
}Servo;

typedef struct {
  Servo servo_0;
  Servo servo_1;
  Servo servo_2;
  Servo servo_3;
  float length_0;
  float length_1;
  float length_2;
}Arm;

typedef struct {
  float target_X;
  float target_Y;
  float target_Z;
}Positsion;

#endif // SERVO_H