#include <Servo.h>
Servo servo;  // create Servo object to control a servo
Servo servo2;
float lastval[2]={0,0};
const float cf = 0.25;
int cotEsq=0;
int ombEsq=0;



void write_quant_deg(Servo &sel_servo, float target_val, int pos){
    int i = lastval[pos];

    if(i > target_val){
      for(i; i > target_val; i--){
        sel_servo.write(i);
        Serial.print(i); Serial.print(",");
        delay(1);
      }
    } 

    if(i < target_val){
      for(i; i < target_val; i++){
        sel_servo.write(i);
        Serial.print(i); Serial.print(",");
        delay(1);
      }
    }
    sel_servo.write(target_val);
    lastval[pos] = i;

}

void setup() {
  servo.attach(9);  // attaches the servo on pin 9 to the Servo object
  servo2.attach(11);
  Serial.begin(115200);
}

void loop() {
  if(Serial.available()){
    int cotEsq_read = Serial.parseInt();  
    int ombEsq_read = Serial.parseInt();                                                                                                                                                                                                                                                                                                         
    cotEsq_read = constrain(cotEsq_read,0,180);
    ombEsq_read = constrain(ombEsq_read,0,180);

    write_quant_deg(servo,cotEsq_read, 0);
    write_quant_deg(servo2, ombEsq_read, 1);
  }
}
