#include "HCPCA9685.h"
#include "servo.h"
#include "config.h"
 
/* I2C slave address for the device/module. For the HCMODU0097 the default I2C address
   is 0x40 */
#define  I2CAdd 0x40

/* Create an instance of the library */
HCPCA9685 HCPCA9685(I2CAdd);

int flag = 0;
int input;
void setup() 
{
  Serial.begin(115200);
  while (!Serial);
  if(setup_servo(Jarvis) >= 1){
    Serial.println("Error in Setup");
    while (true){
      delay(1000000);        
    }
  }
  else{
    Serial.println("Jarvis ready 2 Rumbel!");
  }
}
 
 
void loop(){
  kinematic_check();
  update_all(Jarvis);
  delay(50);
  if(flag == 0){ 
  Postsions.target_X= Postsions.target_X + 2;  

  }
  if(flag == 1){ 
  
  Postsions.target_X= Postsions.target_X - 2;

  }
  if(Postsions.target_X >= 100)  {
    flag = 1;
  }
  if(Postsions.target_X < -100){
    flag = 0;    
  }
  


}
