#include <avr/pgmspace.h>
#include <LiquidCrystal.h>
#include <PID_v1.h>
#include "thermistortables.h"
/*
 * LCD RS pin to digital pin 38
 * LCD Enable pin to digital pin 40
 * LCD D4 pin to digital pin 36
 * LCD D5 pin to digital pin 34
 * LCD D6 pin to digital pin 32
 * LCD D7 pin to digital pin 30
 */
LiquidCrystal lcd(38, 40, 36, 34, 32, 30);

#define h1t A7
#define h2t A6
#define h3t A5
#define h4t A4
#define p1t A3
#define p2t A2
#define m1t A1
#define m2t A0

#define H1 2
#define H2 3
#define H3 4
#define H4 5
#define P1 6
#define P2 7
#define aux1 8
#define aux2 9

#define enc_button 20
#define enc2 19
#define enc3 18

unsigned long serialTime; //this will help us know when to talk with processing

static const uint8_t analog_pins[] = {A0, A1, A2, A3, A4, A5, A6, A7}; //variable to allow a for loop to read from analog port

//variables for PID control
double therm_set[8];
double therm_act[8];
double therm_pwm[8];

double therm_p[8];
double therm_i[8];
double therm_d[8];

int topMenu = 0;
int subMenu = 0;

volatile boolean buttonFlag = false;
volatile int rotaryCount = 0;

volatile boolean enc2_flag = false;
volatile boolean enc3_flag = false;


PID therm1_PID(&therm_act[0], &therm_pwm[0], &therm_set[0], therm_p[0], therm_i[0], therm_d[0], DIRECT);//Heaters 1,4 are acting on the same thermistor (therm 0) in one half of the mould
PID therm2_PID(&therm_act[0], &therm_pwm[1], &therm_set[0], therm_p[1], therm_i[1], therm_d[1], DIRECT);//Heaters 1,2 are acting on the same thermistor (therm 0) in one half of the mould
PID therm3_PID(&therm_act[6], &therm_pwm[2], &therm_set[0], therm_p[2], therm_i[2], therm_d[2], DIRECT);//Heaters  is the inner mandrel of the hot block connected to T7 
PID therm4_PID(&therm_act[1], &therm_pwm[3], &therm_set[0], therm_p[3], therm_i[3], therm_d[3], DIRECT);//Heater  are acting on the same thermistor (therm 1) AND the same heater output in the second half of the mould
PID therm5_PID(&therm_act[5], &therm_pwm[4], &therm_set[1], therm_p[4], therm_i[4], therm_d[4], REVERSE);
PID therm6_PID(&therm_act[3], &therm_pwm[5], &therm_set[1], therm_p[5], therm_i[5], therm_d[5], REVERSE);
//PID therm7_PID(&therm_act[6], &therm_pwm[6], &therm_set[6], therm_p[6], therm_i[6], therm_d[6], DIRECT);
//PID therm8_PID(&therm_act[7], &therm_pwm[7], &therm_set[7], therm_p[7], therm_i[7], therm_d[7], DIRECT);

//variables for use in a software timer
unsigned long enc2_ct;
unsigned long enc2_pt=millis(); 
unsigned long enc3_ct;
unsigned long enc3_pt=millis(); 
unsigned long current_time=0;
unsigned long previous_time=millis();
//=================================================================================================================
//=================================================================================================================
void setup() {
  Serial.begin(9600);
  // set up the LCD's number of columns and rows:
  lcd.begin(20, 4);
  // Print a message to the LCD.
  lcd.print("Booting.....");
  
  for(int i=2; i<10; i++){ //set temperature control switches to ouputs
    pinMode(i, OUTPUT);
    pinMode(analog_pins[i-2], INPUT);
  }


  
  pinMode(enc_button, INPUT_PULLUP);
  pinMode(enc2, INPUT_PULLUP);
  pinMode(enc3, INPUT_PULLUP);

  digitalWrite(8, HIGH);//switch fans on
  digitalWrite(9, HIGH); //switch fans on


  therm1_PID.SetOutputLimits(0, 255); // set to 100 for testing
  therm2_PID.SetOutputLimits(0, 255);
  therm3_PID.SetOutputLimits(0, 255);
  therm4_PID.SetOutputLimits(0, 255);
  therm5_PID.SetOutputLimits(0, 255);
  therm6_PID.SetOutputLimits(0, 255);
  //therm7_PID.SetOutputLimits(0, 255);
  //therm8_PID.SetOutputLimits(0, 255);

  therm1_PID.SetMode(AUTOMATIC);
  therm2_PID.SetMode(AUTOMATIC);
  therm3_PID.SetMode(AUTOMATIC);
  therm4_PID.SetMode(AUTOMATIC);
  therm5_PID.SetMode(AUTOMATIC);
  therm6_PID.SetMode(AUTOMATIC);
  //therm7_PID.SetMode(AUTOMATIC);
  //therm8_PID.SetMode(AUTOMATIC);

  therm1_PID.SetTunings(120.0, 1.0, 1.0);
  therm2_PID.SetTunings(120.0, 1.0, 1.0);
  therm3_PID.SetTunings(120.0, 1.0, 1.0);
  therm4_PID.SetTunings(120.0, 1.0, 1.0);
  
  therm5_PID.SetTunings(120.0, 1.0, 1.0);
  therm6_PID.SetTunings(120.0, 1.0, 1.0);

  
  therm_set[0]=20.0;
  therm_set[1]=5.0;

  attachInterrupt(digitalPinToInterrupt(enc_button), encoder_button, RISING);
    
}
//=================================================================================================================
//=================================================================================================================
void loop() {
 // Serial.println(analogRead(A0));

  current_time=millis();
  if(previous_time-current_time>100){//update PID and display every 100ms
    previous_time=current_time;
    get_temp();
    therm1_PID.Compute();
    therm2_PID.Compute();
    therm3_PID.Compute();
    therm4_PID.Compute();
    therm5_PID.Compute();
    //therm6_PID.Compute();
    update_outputs();
    display_temp();
  }
  if(buttonFlag){
    menu_render();
  }

  //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
}
//=================================================================================================================
//=================================================================================================================
//read analog inputs and lookup temperature from table.
void get_temp(){
  int an_in=0;
  for(int c=0; c<8; c++){ //loop over analog inputs
    for(int i=0; i<OVERSAMPLENR; i++){ //take OVERSAMPLENR readings on each input
      delay(1);
      an_in += analogRead(analog_pins[c]);
    } 
    therm_act[c] = temp_lookup(an_in);
    an_in=0;
  }
  return;
}
//=================================================================================================================
//this function looks for the nearest value in the temptable and does a linear map to the next.
int temp_lookup(int adc_val){
  for(int i=0; i<50; i++){
    if(adc_val<temptable[i][0]){
      int temprange = map(adc_val, temptable[i-1][0], temptable[i][0], temptable[i-1][1], temptable[i][1]);
      return temprange;
    }
  }
  return 0; 
}
//=================================================================================================================
void display_temp(){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("T1:");
    lcd.print(int(therm_act[0]));
    lcd.setCursor(8,0);
    lcd.print("T2:");
    lcd.print(int(therm_act[1]));
    lcd.setCursor(0,1);
    lcd.print("T3:");
    lcd.print(int(therm_act[2]));
    lcd.setCursor(8,1);   
    lcd.print("T4:");
    lcd.print(int(therm_act[3]));
    lcd.setCursor(0,2);
    lcd.print("T5:");
    lcd.print(int(therm_act[4]));
    lcd.setCursor(8,2);
    lcd.print("T6:");
    lcd.print(int(therm_act[5]));
    lcd.setCursor(0,3);
    lcd.print("T7:");
    lcd.print(int(therm_act[6]));
    lcd.setCursor(8,3);
    lcd.print("T8:");
    lcd.print(int(therm_act[7]));
  return;
}
//=================================================================================================================
void update_outputs(){
  for(int j=2; j<8; j++){//exclude outputs 7 and 8 which are connected to the peltier fans.
    analogWrite(j, therm_pwm[j-2]); // write the pwm values to digital pins 2-9
  }  
  return;
}
//=================================================================================================================
//=================================================================================================================
void encoder_button(){
   buttonFlag=true;
  return;
}

//=================================================================================================================
void encoder_2(){
  
  enc2_ct=millis();
  if(enc2_ct-enc2_pt>50){
    if(digitalRead(enc3)==HIGH){
      rotaryCount-=1;
    }
   }
  enc2_pt=enc2_ct;
  return;
}
//=================================================================================================================
void encoder_3(){
  enc3_ct=millis();
  if(enc3_ct-enc3_pt>50){
        if(digitalRead(enc2)==HIGH){
          rotaryCount+=1;
        }
    }
  enc3_pt=enc3_ct;
  return;
}
//=================================================================================================================
//=================================================================================================================
//=================================================================================================================
void menu_render(){
  
    if(buttonFlag){
        buttonFlag=false;
        subMenu+=1;
    }
    if(subMenu>=3){
      subMenu=0;
    }
    
    switch(subMenu){
      case 0:
        tempChanger();
        break;
      case 1:
        tempChanger();
        break;
    }

  return;
}

void tempChanger(){
  attachInterrupt(digitalPinToInterrupt(enc2), encoder_2, FALLING);
  attachInterrupt(digitalPinToInterrupt(enc3), encoder_3, FALLING);
  int target_temp = therm_set[0];
  rotaryCount=target_temp;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Set T1");
  while(!buttonFlag){
    
    delay(150);
    if(rotaryCount>300){
      rotaryCount=0;
    }
    if(rotaryCount<0){
      rotaryCount=300;
    }
    
    //noInterrupts();
    delay(5);
    lcd.clear();
    delay(5);
    lcd.setCursor(0,0);
    delay(5);
    lcd.print("Set T1");
    delay(5);
    lcd.setCursor(0,1);
    delay(5);
    lcd.print(rotaryCount);
    delay(5);
    //interrupts();

  }
  detachInterrupt(digitalPinToInterrupt(enc2));
  detachInterrupt(digitalPinToInterrupt(enc3));
  Serial.println("done with temp change");
  return;
}

/********************************************
 * Serial Communication functions for PID tuning with a processing frontend to graph the output
 ********************************************/


union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array



// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float setpoint
//  6-9: float input
//  10-13: float output  
//  14-17: float P_Param
//  18-21: float I_Param
//  22-245: float D_Param
void SerialReceive()
{

  // read the bytes sent from Processing
  int index=0;
  byte Auto_Man = -1;
  byte Direct_Reverse = -1;
  while(Serial.available()&&index<26)
  {
    if(index==0) Auto_Man = Serial.read();
    else if(index==1) Direct_Reverse = Serial.read();
    else foo.asBytes[index-2] = Serial.read();
    index++;
  } 
  
  // if the information we got was in the correct format, 
  // read it into the system
  if(index==26  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1))
  {
    therm_set[0]=double(foo.asFloat[0]);
    //therm_act[0]=double(foo.asFloat[1]);       // * the user has the ability to send the 
                                          //   value of "Input"  in most cases (as 
                                          //   in this one) this is not needed.
    if(Auto_Man==0)                       // * only change the output if we are in 
    {                                     //   manual mode.  otherwise we'll get an
      therm_pwm[0]=double(foo.asFloat[2]);      //   output blip, then the controller will 
    }                                     //   overwrite.
    
    double p, i, d;                       // * read in and set the controller tunings
    p = double(foo.asFloat[3]);           //
    i = double(foo.asFloat[4]);           //
    d = double(foo.asFloat[5]);           //
    therm1_PID.SetTunings(p, i, d);            //
    
    if(Auto_Man==0) therm1_PID.SetMode(MANUAL);// * set the controller mode
    else therm1_PID.SetMode(AUTOMATIC);             //
    
    if(Direct_Reverse==0) therm1_PID.SetControllerDirection(DIRECT);// * set the controller Direction
    else therm1_PID.SetControllerDirection(REVERSE);          //
  }
  Serial.flush();                         // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
  Serial.print("PID ");
  Serial.print(therm_set[0]);   
  Serial.print(" ");
  Serial.print(therm_act[0]);   
  Serial.print(" ");
  Serial.print(therm_pwm[0]);   
  Serial.print(" ");
  Serial.print(therm1_PID.GetKp());   
  Serial.print(" ");
  Serial.print(therm1_PID.GetKi());   
  Serial.print(" ");
  Serial.print(therm1_PID.GetKd());   
  Serial.print(" ");
  if(therm1_PID.GetMode()==AUTOMATIC) Serial.print("Automatic");
  else Serial.print("Manual");  
  Serial.print(" ");
  if(therm1_PID.GetDirection()==DIRECT) Serial.println("Direct");
  else Serial.println("Reverse");
}



