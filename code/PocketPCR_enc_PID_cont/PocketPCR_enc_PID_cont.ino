/*
  Analog input, analog output, serial output

  Reads an analog input pin, maps the result to a range from 0 to 255 and uses
  the result to set the pulse width modulation (PWM) of an output pin.
  Also prints the results to the Serial Monitor.

  The circuit:
  - potentiometer connected to analog pin 0.
    Center pin of the potentiometer goes to the analog pin.
    side pins of the potentiometer go to +5V and ground
  - LED connected from digital pin 9 to ground

  created 29 Dec. 2008
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogInOutSerial
*/

#include <math.h> 
#include "Adafruit_GFX.h"
#include <Adafruit_SSD1306.h>
#include "Rotary.h"

#include <Fonts/FreeSans9pt7b.h>

// These constants won't change. They're used to give names to the pins used:
const int analogInPin = A2;  // Analog input pin that the potentiometer is attached to
const int fanPin = 4; // Analog output pin that the LED is attached to
const int heaterPin = 8; // Analog output pin that the LED is attached to
const int lidPin = 9; // Analog output pin that the LED is attached to
const int butPin = A4; // Analog output pin that the LED is attached to

const int NTC_B = 3435;
const float NTC_TN = 298.15;
const int NTC_RN = 10000;
const float NTC_R0 = 4.7;

const float temperature_tollerance=0.5;
// Rotary encoder is wired with the common to ground and the two
// outputs to pins 2 and 3.
#define ENCODER_1 6
#define ENCODER_2 7
Rotary rotary = Rotary(ENCODER_1, ENCODER_2);

// If using software SPI (the default case):
#define OLED_MOSI   20
#define OLED_CLK   21
#define OLED_DC    0 //11
#define OLED_CS    22 //12
#define OLED_RESET 5

#define CASE_Main 1
#define CASE_Settings 2
#define CASE_EditSettings 3
#define CASE_Run 4

#define PCR_set 1
#define PCR_transition 2
#define PCR_time 3
#define PCR_end 4



#define PIDp 0.6
#define PIDi 1
#define PIDd 0.2

#define SETTING_Size 5
String SETTING_String[SETTING_Size] = { "Init", "Denature", "Annealing" , "Extension", "Final" };
int SETTING_Value[SETTING_Size*2];
int SETTING_cycles = 20;

Adafruit_SSD1306 display( OLED_MOSI,OLED_CLK,OLED_DC, OLED_RESET, OLED_CS);

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
float temperature =0;
float temperature_mean =0;

float sensorResistance =0;
float sensorVoltage =0;

boolean editMode = false;
boolean minuteMode = false;

int counter = 0;        // value read from the pot

int x=0;
int caseUX = 1; 
int casePCR = 1;
int MenuItem = 1;
int PCRstate = 0;
int PCRcycle = 1;

int PCRpwm = 0;

float TEMPset;
float TEMPdif;
float TEMPcontrol;
float TEMPdif_a[10];
float TEMPd;
long TEMPclick=0;
long TIMEclick=0;
int TIMEcontrol=0;

int heatPower = 191;
int fanPower=0;
void setup() {

  pinMode(ENCODER_1, INPUT_PULLUP);
  pinMode(ENCODER_2, INPUT_PULLUP);
  pinMode(butPin, INPUT_PULLUP);

  pinMode(fanPin, OUTPUT);
  pinMode(heaterPin, OUTPUT);
  pinMode(lidPin, OUTPUT);

  attachInterrupt(ENCODER_1, rotate, CHANGE);
  attachInterrupt(ENCODER_2, rotate, CHANGE);

  // initialize serial communications at 9600 bps:
  Serial.begin(115200);

SETTING_Value[0] = 99;      
SETTING_Value[1] = 60;   

SETTING_Value[2] = 95;      
SETTING_Value[3] = 120;  

SETTING_Value[4] = 55;      
SETTING_Value[5] = 80;  

SETTING_Value[6] = 72;      
SETTING_Value[7] = 40;  

SETTING_Value[8] = 25;      
SETTING_Value[9] = 00;  


    display.begin(SSD1306_SWITCHCAPVCC);

      // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.

  display.clearDisplay();  
    
 display.setTextSize(1);
 display.setCursor(1,55);
  display.setTextColor(WHITE);
  display.println("V3.2");
      display.dim(false);
  display.display();
   // display.setFont(&FreeMono9pt7b);

   analogReadResolution(12);


    
}

void loop() {
  // read the analog in value:
 // analogReference(AR_EXTERNAL);

analogWrite(fanPin,0);
analogWrite(heaterPin,0);
    delay(20);
sensorValue = analogRead(analogInPin);
  // map it to the range of the analog out:
sensorVoltage=3.3*sensorValue/4096;
sensorResistance=((sensorVoltage*NTC_R0)/(3.3-sensorVoltage));
temperature=  1/(log(sensorResistance*1000/NTC_RN)/NTC_B+1/NTC_TN)-273.15 ;
temperature_mean= (temperature_mean*3+temperature)/4;

  // print the results to the Serial Monitor:
  //Serial.print("sensor = ");
  //  Serial.print(sensorValue);
 // Serial.print("\t output = ");
 // Serial.println(temperature);


      
  // wait 2 milliseconds before the next loop for the analog-to-digital
  // converter to settle after the last reading:
 


if (temperature>105) {
  heatPower=0;
  fanPower=63;
  };
  

     // digitalWrite(heaterPin, HIGH);   // turn the LED on (HIGH is the voltage level)

    //  analogWrite(heaterPin, heatPower);   // turn the LED on (HIGH is the voltage level)
    //  analogWrite(fanPin, fanPower);   // turn the LED on (HIGH is the voltage level)





switch (caseUX) {

case CASE_Main:

  
   if (counter>1) counter=0;
   if (counter<0) counter=1;
   MenuItem = counter;
   draw_main_display();
   
   if (!digitalRead(butPin)) {while (!digitalRead(butPin));
   
   if (MenuItem==0) {caseUX=CASE_Run; PCRstate=0; counter=0;}
   if (MenuItem==1) {caseUX=CASE_Settings; counter=0;}

   }
   
    break;
    
case CASE_Settings:

    if (counter>11) counter=0;
    if (counter<0) counter=11;
    MenuItem = counter;
    if ((MenuItem%2==1)&&(SETTING_Value[MenuItem]>90)) minuteMode=true; else minuteMode=false;
    if (!digitalRead(butPin)) 
    {while (!digitalRead(butPin)); caseUX=CASE_EditSettings; 
    if (minuteMode) counter=SETTING_Value[MenuItem]/60; else counter=SETTING_Value[MenuItem];}
    draw_setup_display();
    
    break;

case CASE_EditSettings:

    
    if (MenuItem%2==0)
     {if (counter<25) counter=25;
      if (counter>99) {counter=99;}

     } // Temp Menu
     
     else
      {if (minuteMode)
       {if (counter<2) {minuteMode=false; counter=90;}
       }  //Minut Mode
       else
       {
       if (counter>90) {minuteMode=true; counter=2;}
       if (counter<0) {counter=0;}
       } //not Minut Mode
      } // Time Menu
    
    if (MenuItem==10) caseUX=CASE_Main;
    if (MenuItem<10) {if (minuteMode) SETTING_Value[MenuItem]=counter*60; else SETTING_Value[MenuItem]=counter;}
    if (!digitalRead(butPin)) {while (!digitalRead(butPin)); caseUX=CASE_Settings;counter=MenuItem;}
    draw_setup_display();
    
    break;

case CASE_Run:

  TEMPdif=TEMPset-temperature_mean;

  if (millis()-TEMPclick>200) {
  TEMPclick=millis();
  TEMPd=TEMPdif_a[4]-TEMPdif;
  TEMPdif_a[4]=TEMPdif_a[3];
  TEMPdif_a[3]=TEMPdif_a[2];
  TEMPdif_a[2]=TEMPdif_a[1];
  TEMPdif_a[1]=TEMPdif_a[0];
  TEMPdif_a[0]=TEMPdif;
    Serial.println (TEMPd);
  }
  
    switch (casePCR) {

    case PCR_set:
    TEMPset=SETTING_Value[PCRstate*2];
    TIMEcontrol=SETTING_Value[1+PCRstate*2];
    casePCR=PCR_transition;
    break;

#define PCR_set 1
#define PCR_transition 2
#define PCR_time 3
#define PCR_end 4

    case PCR_transition:
    
    runPID();
    draw_run_display();
    
    if (abs(TEMPset-temperature_mean)<temperature_tollerance) {TIMEclick=millis(); casePCR=PCR_time;}
    
    break;

    case PCR_time:

    runPID();
    TIMEcontrol=SETTING_Value[1+PCRstate*2]-(millis()-TIMEclick)/1000;
    draw_run_display();

    if (TIMEcontrol<=0) {
    if (PCRstate==3)
    {PCRstate=1;
    PCRcycle++;
    } else
    {PCRstate++;casePCR=PCR_set;};
    casePCR=PCR_set;
    }
    break;
    
    default:
    // Statement(s)
    break;
    }//PCR switch

    break;

default:
    // Statement(s)
    break;
    
} //switch
}




void draw_value_box(int x, int y, int val, bool highlight,bool edit)
{

  display.drawRect(x-3,y+2-val/2,17,val/2-1,1);
  
if (highlight&&!edit){
  display.fillRect(x-3,y+2-val/2,17,11,1);
  display.setTextColor(0);
}
if (highlight&&edit)  display.drawRect(x-2,y+2-val/2,15,11,1);

 display.setCursor(x,y-val/2+4);
 display.println(val);
 display.setTextColor(1);
};

void draw_value(int x, int y, int val, bool highlight,bool edit)
{
if (highlight&&!edit){
  display.fillRect(x-3,y-1,17,12,1);
  display.setTextColor(0);
    display.setCursor(x,y-1);

  } else   display.setCursor(x,y);

 //display.drawRect(x-3,y,2,7,1);
 //display.drawRect(x+12,y,2,7,1);
if (highlight&&edit){
 display.drawLine(x-3,y,x-3,y+6,1);
 display.drawLine(x+13,y,x+13,y+6,1);
}

 // display.drawRect(x-3,y-2,17,11,1);
  
  if (val>90) 
   { display.print(val/60); 
   display.println("m");} else
   {
    display.println(val); 
   }
   display.setTextColor(1);
}

void draw_setup_display()

{
  
display.clearDisplay(); 

 if (MenuItem<10) {
  
display.setCursor(5,0);
//display.print("Set ");
display.print(SETTING_String[MenuItem/2]);
if (MenuItem%2==0) {display.print(" Temp. [");display.print("\xA7");display.print("C]");} else {display.print(" Time "); if (minuteMode) display.print("[min]"); else display.print("[s]");  }

 } else 

 {
 if (MenuItem==11)   // Set or Cancel
 {

 display.setCursor(34,0);
 display.println("Set");
 display.fillRect(70,0,42,9,1);
 display.setTextColor(0);
 display.setCursor(75,1);
 display.println("Cancel");
 display.setTextColor(1);
 } else

 {
  display.fillRect(30,0,25,9,1);
 display.setTextColor(0);
 display.setCursor(34,1);
 display.println("Set");
 display.setTextColor(1);
 display.setCursor(75,0);
 display.println("Cancel");
 
  }
}


draw_value_box (21,55,SETTING_Value[0],MenuItem==0,caseUX==CASE_EditSettings);
draw_value_box (43,55,SETTING_Value[2],MenuItem==2,caseUX==CASE_EditSettings);
draw_value_box (63,55,SETTING_Value[4],MenuItem==4,caseUX==CASE_EditSettings);
draw_value_box (83,55,SETTING_Value[6],MenuItem==6,caseUX==CASE_EditSettings);
draw_value_box (105,55,SETTING_Value[8],MenuItem==8,caseUX==CASE_EditSettings);

draw_value (21,57,SETTING_Value[1],MenuItem==1,caseUX==CASE_EditSettings);
draw_value (43,57,SETTING_Value[3],MenuItem==3,caseUX==CASE_EditSettings);
draw_value (63,57,SETTING_Value[5],MenuItem==5,caseUX==CASE_EditSettings);
draw_value (83,57,SETTING_Value[7],MenuItem==7,caseUX==CASE_EditSettings);
draw_value (105,57,SETTING_Value[9],MenuItem==9,caseUX==CASE_EditSettings);

   

  display.display();
  
 // Serial.println (encoder0Pos, DEC);

    // Serial.println(Enc_counter,DEC);
   // Serial.print(millis());
   //    Serial.print(" :");
  //   Serial.println(temperature,DEC);
  
  
  }

void draw_main_display()
{
display.clearDisplay(); 

   display.setFont(&FreeSans9pt7b);


   if (MenuItem==0)   // Set or Cancel
 {

  display.fillRect(21,10,86,25,1);
 display.setTextColor(0);
 display.setCursor(26,28);
 display.println("Run PCR");

 //display.fillRect(21,38,86,25,1);
 display.setTextColor(1);
 display.setCursor(40,55);
 display.println("Setup");
 } else

 {
 // display.fillRect(21,10,86,25,1);
 display.setTextColor(1);
 display.setCursor(26,28);
 display.println("Run PCR");

 display.fillRect(21,38,86,25,1);
 display.setTextColor(0);
 display.setCursor(40,55);
 display.println("Setup");
 
  }
  display.setFont();
    display.display();

  }


void draw_run_display()

{
  display.clearDisplay(); 

 display.fillRect(0,0,128,11,1);
  display.setCursor(26,2);
 display.setTextColor(0);
 display.println("PCR Running");
 display.setTextColor(1);

 
  display.setCursor(0,14);

display.print("State: ");
 display.print(SETTING_String[PCRstate]);
 display.setCursor(0,23);
 display.print("Cycle #: ");
 display.print(PCRcycle);
 
 display.drawLine(0,34,128,34,1);

 display.setCursor(0,55-18);
 display.print("Set Temp: ");
 display.print(SETTING_Value[PCRstate*2]);
 display.print(" \xA7");display.print("C");

 display.setCursor(0,55-9);
 display.print("Block Temp: ");
 display.print(temperature_mean);
 display.print(" \xA7");display.print("C");
 display.setCursor(0,55);
 display.print("Time: ");
  display.print(TIMEcontrol);
 display.print(" ");
 display.print(TEMPcontrol);
 display.print(" ");
  display.print(heatPower);




// display.print();
 display.print(" s");




 display.setCursor(1,25);
 display.drawLine(x,10*(temperature_mean-25),x,10*(temperature_mean-25),1);

 display.display();




   
  }

// rotate is called anytime the rotary inputs change state.
void rotate() {
  delayMicroseconds(500) ;

  unsigned char result = rotary.process();
  if (result == DIR_CW) {
    counter++;
    Serial.println(counter);
  } else if (result == DIR_CCW) {
    counter--;
    Serial.println(counter);
  }
}

int power_heating(float temperature,float power) 
{
float power_return;

#define FA_c 0.0029843
#define FA_d 0.1604843
#define FA_e 0.00001004
#define FA_f 0.0044208

power_return = (FA_d-FA_f*temperature-power)/(FA_e*temperature-FA_c);

if (power_return>255) power_return=255;
if (power_return<0) power_return=0;

return (int)power_return;

}
  
int power_cooling(float temperature,float power) 
{
float power_return;
  
#define FA_i 0.00011045
#define FA_j 0.0052367
#define FA_k 0.002884
#define FA_l 0.1891699

power_return =(FA_l-FA_j*temperature-power)/(FA_i*temperature-FA_k);

if (power_return>255) power_return=255;
if (power_return<0) power_return=0;

return (int)power_return;
}

void runPID()
{
   TEMPcontrol= PIDp*TEMPdif+PIDd*TEMPd;

  if (TEMPcontrol>0)
  {
    heatPower=power_heating(temperature_mean,TEMPcontrol);
  analogWrite(fanPin,0);
  analogWrite(heaterPin,heatPower);
  }
  else 
  {
    heatPower=power_cooling(temperature_mean,TEMPcontrol);
  analogWrite(fanPin,heatPower);
  analogWrite(heaterPin, 0);
    } 
  }
