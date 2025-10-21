#include <Servo.h>
Servo servo;  // create Servo object to control a servo
Servo servo2;
float lastval[2]={0,0};
const float cf = 0.25;
int cotEsq=0;
int ombEsq=0;

float suavise(int var, int pos){
  return (var*cf) + (1-cf)*lastval[pos];
}

void write_quant_deg(Servo &sel_servo, float actual_val, int pos){
    float lstval_to_alter = lastval[pos];

    if(lstval_to_alter > actual_val){
      for(lstval_to_alter; lstval_to_alter > actual_val; lstval_to_alter--){
        sel_servo.write(lstval_to_alter);
        lastval[pos] = lstval_to_alter;
        delay(1);
      }
    } 

    if(lstval_to_alter < actual_val){
      for(lstval_to_alter; lstval_to_alter < actual_val; lstval_to_alter++){
        sel_servo.write(lstval_to_alter);
        lastval[pos] = lstval_to_alter;
        delay(1);
      }
    }
    lastval[pos] = actual_val;
}

void setup() {
  servo.attach(9);  // attaches the servo on pin 9 to the Servo object
  servo2.attach(11);
  Serial.begin(9600);
}

void loop() {
  if(Serial.available()){
    int cotEsq_read = Serial.parseInt();  
    int ombEsq_read = Serial.parseInt();      
                                                                                                                                                                                                                                                                                                                                   
    cotEsq_read = constrain(cotEsq_read,0,180);
    ombEsq_read = constrain(ombEsq_read, 0, 70);

    float cotEsq_suave = suavise(cotEsq_read, 0);
    float ombEsq_suave = suavise(ombEsq_read, 1);

    write_quant_deg(servo, cotEsq_suave, 0);
    write_quant_deg(servo2, ombEsq_suave, 1);

    delay(15);
  }
}



