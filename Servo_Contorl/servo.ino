#include "servo.h"
#include "HCPCA9685.h"

//Defining Constants
#define PI 3.1415
#define rad2degr (180/3.1415)
#define degr2rad (3.1415/180)

// Updateing Set Poston
void set_Pos_Angular(Servo &servo){
  // Checking if set angel is in Kinematic Posibil Range
  if(servo.set_angel >= servo.max_angel){
    Serial.print("Servo: ");
    Serial.println(servo.servo_ID);
    Serial.println("Angel is set 2 uper boundry");
    servo.set_angel = servo.max_angel;
  }
  if(servo.set_angel <= servo.min_angel){
    Serial.print("Servo: ");
    Serial.println(servo.servo_ID);    
    Serial.println("Angel is set 2 lower boundry");
    servo.set_angel = servo.min_angel;
  }
  // Maping the Angel onto the Puls Width
  float pos = map(servo.set_angel, servo.max_angel, servo.min_angel, servo.map_min, servo.map_max);
  // Writing the Pulswidth on to the Servochanel
  HCPCA9685.Servo(servo.servo_ID, pos);
}

int setup_servo(Arm &arm){
  if(check_config(arm.servo_0) || check_config(arm.servo_1) || check_config(arm.servo_2) || check_config(arm.servo_3)){
      HCPCA9685.Init(SERVO_MODE);
      HCPCA9685.Sleep(false);
      return 0;
  }
  else{
    return 1;
  }
  
}

bool check_config(Servo &servo){
  if(servo.max_angel > servo.min_angel){
    return true;    
  }
  else{
    return false;
  }  
}

void update_all(Arm &arm){
  set_Pos_Angular(arm.servo_0);
  set_Pos_Angular(arm.servo_1);
  set_Pos_Angular(arm.servo_2);
  set_Pos_Angular(arm.servo_3);    
}

void forward_kinematic(Arm &arm, Positsion &positsion){
  float x_pos_helper_1 = arm.length_1 * cos(arm.servo_1.set_angel * degr2rad);
  float y_pos_helper_1 = arm.length_1 * sin(arm.servo_1.set_angel * degr2rad);
  float angel_3_helper_1 = (arm.servo_2.set_angel * degr2rad) + (arm.servo_1.set_angel * degr2rad) - 180;
  float x_pos_helper_2 = arm.length_1 * cos(angel_3_helper_1);
  float y_pos_helper_2 = arm.length_1 * sin(angel_3_helper_1);
  float x_pos_helper_3 = x_pos_helper_1 + x_pos_helper_2;
  positsion.target_Y = y_pos_helper_1 + y_pos_helper_2;
  positsion.target_X = x_pos_helper_3 * cos(arm.servo_0.set_angel);
  positsion.target_Z = x_pos_helper_3 * sin(arm.servo_0.set_angel);
}

void invers_kinematic(Arm &arm, Positsion &positsion){
  float cubed_X = (positsion.target_X * positsion.target_X);
  float cubed_Y = (positsion.target_Y * positsion.target_Y);
  float cubed_Z = (positsion.target_Z * positsion.target_Z);
  float length_3 = sqrt(cubed_X + cubed_Y + cubed_Z);
  arm.servo_2.set_angel = (acos(((length_3 * length_3) - (arm.length_2 * arm.length_2) - (arm.length_1 * arm.length_1)) /(-2* length_3 * arm.length_1)))*rad2degr;
  float angel_2_helper_1 = asin(positsion.target_X/length_3)*rad2degr;
  float angel_2_helper_2 = asin(sin((arm.servo_3.set_angel*degr2rad)/(length_3 * arm.length_2)))*rad2degr;
  arm.servo_1.set_angel = angel_2_helper_1 + angel_2_helper_2;
  arm.servo_0.set_angel = atan(positsion.target_Z/positsion.target_X )*rad2degr;
}

float cubed(float input){ return (input*input);}

void invers_kinematic_2(Arm &arm, Positsion &positsion){
    arm.servo_0.set_angel = atan(positsion.target_Y/positsion.target_X) * rad2degr;
    float r_1 = sqrt(cubed(positsion.target_X) + cubed(positsion.target_Y));
    float r_2 = positsion.target_Z - arm.length_0;
    float helper_angel_2 = atan(r_2 / r_1) * rad2degr;
    float r_3 = sqrt(cubed(r_1) + cubed(r_2));
    float helper_angel_1 = acos((cubed(arm.length_2) - cubed(arm.length_1) - cubed(r_3)) / (-2 * arm.length_1 * r_3)) * rad2degr;
    arm.servo_1.set_angel = helper_angel_2 + helper_angel_1;
    float helper_angel_3 = acos((cubed(r_3) - cubed(arm.length_1) - cubed(arm.length_2)) / (-2 * arm.length_1 * arm.length_2)) * rad2degr;
    arm.servo_2.set_angel = 180 - helper_angel_3;
}

void forward_kinematic_2(Arm &arm, Positsion &positsion){
    float helper_angel_3 = 180 - arm.servo_2.set_angel;
    float r_3 = sqrt(cubed(arm.length_2) + cubed(arm.length_1) - 2 * arm.length_2 * arm.length_1 * cos(helper_angel_3 * degr2rad));
    float helper_angel_1 = (acos((cubed(arm.length_2) - cubed(arm.length_1) - cubed(r_3)) / (-2 * r_3 * arm.length_1)) * rad2degr);
    float helper_angel_2 = arm.servo_1.set_angel - helper_angel_1;
    positsion.target_Z = arm.length_0 + (r_3 * sin(helper_angel_2 * degr2rad));
    float r_1 = r_3 * cos(helper_angel_2 * degr2rad);
    positsion.target_X = r_1 * cos((arm.servo_0.set_angel * degr2rad));
    positsion.target_Y = r_1 * sin((arm.servo_0.set_angel * degr2rad));
}

void kinematic_check(){
  Serial.print("Servo Angels (A,B;C): ");
  Serial.print(Jarvis.servo_0.set_angel);
  Serial.print(", ");
  Serial.print(Jarvis.servo_1.set_angel);
  Serial.print(", ");
  Serial.println(Jarvis.servo_2.set_angel);
  Serial.print("Arm Pos (X,Y;Z): ");
  Serial.print(Postsions.target_X);
  Serial.print(", ");
  Serial.print(Postsions.target_Y);
  Serial.print(", ");
  Serial.println(Postsions.target_Z);
  delay(5000);
  Serial. println("Inverse Kinematic:");
  invers_kinematic_2(Jarvis, Postsions);
  Serial.print("Servo Angels (A,B;C): ");
  Serial.print(Jarvis.servo_0.set_angel);
  Serial.print(", ");
  Serial.print(Jarvis.servo_1.set_angel);
  Serial.print(", ");
  Serial.println(Jarvis.servo_2.set_angel);

  delay(5000);
  Serial.println("Forward Kinematic:");
  forward_kinematic_2(Jarvis, Postsions);

   Serial.print("Arm Pos (X,Y;Z): ");
  Serial.print(Postsions.target_X);
  Serial.print(", ");
  Serial.print(Postsions.target_Y);
  Serial.print(", ");
  Serial.println(Postsions.target_Z);
  delay(5000);
  }