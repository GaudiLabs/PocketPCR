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
#include "FlashStorage.h"

#include <Fonts/FreeSans9pt7b.h>

const char VersionString[] = "V1.01 2019";

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
#define CASE_Done 5

#define PCR_set 1
#define PCR_transition 2
#define PCR_time 3
#define PCR_end 4


#define PIDp 0.5
#define PIDi 0.0001
#define PIDd 0.15

#define SETTING_Size 6


String SETTING_String[SETTING_Size] = { "Init", "Denature", "Annealing" , "Extension", "Final" };
int SETTING_cycles = 20;

typedef struct {
int value[SETTING_Size*2];
} EEprom;

FlashStorage(my_flash_store, EEprom);
EEprom settings;
  
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
int counter_save = 0;        // value read from the pot

int x=0;
int caseUX = CASE_Main; 
int casePCR = PCR_set;
int MenuItem = 1;
int PCRstate = 0;
int PCRcycle = 1;

int PCRpwm = 0;

float TEMPset;
float TEMPdif;
float TEMPi;
bool PIDIntegration = false;
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

  display.begin(SSD1306_SWITCHCAPVCC);
  
  display.clearDisplay();  
    
  display.setTextSize(1);
  display.setTextColor(WHITE); 
  display.setCursor(15,20);
  display.println("Software Version");
  display.setCursor(30,30);
  display.println(VersionString);
  
  display.setCursor(15,45);
  display.println("Left Handed Mode");
  display.dim(false);
  display.display();
   // display.setFont(&FreeMono9pt7b);


    if (!digitalRead(butPin)) {while (!digitalRead(butPin)); display.setRotation(2);} else   
    display.setRotation(0); // 0 for right handed, 2 for left handed

  // Show image buffer on the display hardware.
  


  analogReadResolution(12);

settings=my_flash_store.read();

if (settings.value[11]==0)
{
settings.value[0] = 94; 
settings.value[1] = 60;   

settings.value[2] = 94;      
settings.value[3] = 120;  

settings.value[4] = 55;      
settings.value[5] = 80;  

settings.value[6] = 72;      
settings.value[7] = 40;  

settings.value[8] = 25;      
settings.value[9] = 180;  

settings.value[10] = 25;  


settings.value[11]=1;
my_flash_store.write(settings);

  };
  
}

void loop() {
  
  // read the analog in value:
 // analogReference(AR_EXTERNAL);

  setHeater(0, 0);
  sensorValue = analogRead(analogInPin);
  setHeater(temperature_mean, TEMPcontrol);

sensorVoltage=3.3*sensorValue/4096;
sensorResistance=((sensorVoltage*NTC_R0)/(3.3-sensorVoltage));
temperature=  1/(log(sensorResistance*1000/NTC_RN)/NTC_B+1/NTC_TN)-273.15 ;
temperature_mean= (temperature_mean*3+temperature)/4;

  // print the results to the Serial Monitor:
  //Serial.print("sensor = ");
  //  Serial.print(sensorValue);
 // Serial.print("\t output = ");
 // Serial.println(temperature);




switch (caseUX) {

case CASE_Main:

  
   if (counter>1) counter=0;
   if (counter<0) counter=1;
   MenuItem = counter;
   draw_main_display();
   
   if (!digitalRead(butPin)) {while (!digitalRead(butPin));
   
   if (MenuItem==0) {caseUX=CASE_Run;casePCR=PCR_set; PCRstate=0; counter=0;}
   if (MenuItem==1) {caseUX=CASE_Settings; counter=0;}

   }
   
    break;
    
case CASE_Settings:

    if (counter>12) counter=0;
    if (counter<0) counter=12;
    MenuItem = counter;
    if ((MenuItem%2==1)&&(settings.value[MenuItem]>90)) minuteMode=true; else minuteMode=false;
    if (!digitalRead(butPin)) 
    {while (!digitalRead(butPin)); caseUX=CASE_EditSettings; 
    if (minuteMode) counter=settings.value[MenuItem]/60; else counter=settings.value[MenuItem];}
    draw_setup_display();
    
    break;

case CASE_EditSettings:

    
    if (MenuItem%2==0)
     {if (MenuItem==10) 
         {if (counter<1) counter=1;
          if (counter>99) counter=99;}
     else 
        {if (counter<25) counter=25;
        if (counter>99) counter=99;}
     
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
    
    if (MenuItem==11) {caseUX=CASE_Main;   my_flash_store.write(settings);}
    if (MenuItem==12) {caseUX=CASE_Main;  settings=my_flash_store.read();}

    if (MenuItem<11) {if (minuteMode) settings.value[MenuItem]=counter*60; else settings.value[MenuItem]=counter;}
    if (!digitalRead(butPin)) {while (!digitalRead(butPin)); caseUX=CASE_Settings;counter=MenuItem;}
    draw_setup_display();
    
    break;

case CASE_Run:

  TEMPdif=TEMPset-temperature_mean;
  TEMPi=TEMPi+(TEMPset-temperature_mean);
  
  if (millis()-TEMPclick>200) {
  TEMPclick=millis();
  TEMPd=TEMPdif_a[4]-TEMPdif;
  TEMPdif_a[4]=TEMPdif_a[3];
  TEMPdif_a[3]=TEMPdif_a[2];
  TEMPdif_a[2]=TEMPdif_a[1];
  TEMPdif_a[1]=TEMPdif_a[0];
  TEMPdif_a[0]=TEMPdif;
  //  Serial.println (TEMPd);
  }
  
    switch (casePCR) {

    case PCR_set:
    TEMPset=settings.value[PCRstate*2];
    TIMEcontrol=settings.value[1+PCRstate*2];
    PIDIntegration=false;
    casePCR=PCR_transition;
    break;


    case PCR_transition:
    runPID();
    draw_run_display();
    if (abs(TEMPset-temperature_mean)<temperature_tollerance) {PIDIntegration=true;TEMPi=0;TIMEclick=millis(); casePCR=PCR_time;}
    break;


    case PCR_time:
    runPID();
    TIMEcontrol=settings.value[1+PCRstate*2]-(millis()-TIMEclick)/1000;
    draw_run_display();

    if (TIMEcontrol<=0) {
      if (PCRstate==4) caseUX=CASE_Done;
    if (PCRstate==3)
    {PCRstate=1;
    PCRcycle++;
    if (PCRcycle>settings.value[10]) PCRstate=4;
    } else
    {PCRstate++;};
    casePCR=PCR_set;
    }
    break;

    
    default:
    // Statement(s)
    break;

    
    }//PCR switch

   break;

case CASE_Done:


  TEMPcontrol=0;
  setHeater(temperature_mean, TEMPcontrol);


    display.clearDisplay(); 
    display.setFont(&FreeSans9pt7b);

    display.fillRect(10,10,100,40,1);
    display.setCursor(15,30);
    display.setTextColor(0);
    display.println("PCR Done");
    display.display(); 
    display.setFont();


if (!digitalRead(butPin)) {while (!digitalRead(butPin)); caseUX=CASE_Main;counter=MenuItem;}

break;
      
default:
    // Statement(s)
    break;
    
} //switch
}




void draw_value_box(int x, int y, int val, bool highlight,bool edit)
{ int shift;

if ((val<46)&&(x>22)&&(x<100)) shift=-13; else shift=0;

  display.drawRect(x-3,y+2-val/2,17,val/2-1,1);
  
if (highlight&&!edit){
  display.fillRect(x-3,y+2-val/2+shift,17,11,1);
  display.setTextColor(0);
}
if (highlight&&edit) display.drawRect(x-2,y+2-val/2+shift,15,11,1);

 display.setCursor(x,y-val/2+4+shift);
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

 if (MenuItem<11) {
  
display.setCursor(5,0);
//display.print("Set ");

if (MenuItem<10){
display.print(SETTING_String[MenuItem/2]);
if (MenuItem%2==0) {display.print(" Temp. [");display.print("\xA7");display.print("C]");} else {display.print(" Time "); if (minuteMode) display.print("[min]"); else display.print("[s]");  }
} else display.print("No of cycles");
 } else 

 {
 if (MenuItem==12)   // Set or Cancel
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


draw_value_box (21,55,settings.value[0],MenuItem==0,caseUX==CASE_EditSettings);
draw_value_box (43,55,settings.value[2],MenuItem==2,caseUX==CASE_EditSettings);
draw_value_box (63,55,settings.value[4],MenuItem==4,caseUX==CASE_EditSettings);
draw_value_box (83,55,settings.value[6],MenuItem==6,caseUX==CASE_EditSettings);
draw_value_box (105,55,settings.value[8],MenuItem==8,caseUX==CASE_EditSettings);


display.drawRect(38,45,61,11,1);
display.fillRect(38+1,45+1,61-2,11-2,0);


if ((MenuItem==10)&&!(caseUX==CASE_EditSettings)){
  display.fillRect(56,45,25,11,1);
  display.setTextColor(0);
}
if ((MenuItem==10)&&(caseUX==CASE_EditSettings))  display.drawRect(56,45,25,11,1);

 display.setCursor(60,47);
 display.print(settings.value[10]);
 display.print("x");
 display.setTextColor(1);

 

draw_value (21,57,settings.value[1],MenuItem==1,caseUX==CASE_EditSettings);
draw_value (43,57,settings.value[3],MenuItem==3,caseUX==CASE_EditSettings);
draw_value (63,57,settings.value[5],MenuItem==5,caseUX==CASE_EditSettings);
draw_value (83,57,settings.value[7],MenuItem==7,caseUX==CASE_EditSettings);
draw_value (105,57,settings.value[9],MenuItem==9,caseUX==CASE_EditSettings);

   

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

   display.setTextColor(1);

    display.display();

  }


void draw_run_display()

{
  display.clearDisplay(); 


 display.fillRect(0,0,128,11,1);
  display.setCursor(16,2);
 display.setTextColor(0);
 if (counter%2==0)  {display.println("  PCR Running");} else
  {display.println("Press to STOP PCR"); if (!digitalRead(butPin)) {while (!digitalRead(butPin)); caseUX=CASE_Done;counter=MenuItem;}
}
 display.setTextColor(1);

 
  display.setCursor(0,14);

display.print("State: ");
 display.print(SETTING_String[PCRstate]);
 display.setCursor(0,23);
 display.print("Cycle: ");
 
 if ((PCRstate>0)&&(PCRstate<4)){
 display.print(PCRcycle);
  display.print(" of ");
  display.print(settings.value[10]);
 }

 
 display.drawLine(0,34,128,34,1);

 display.setCursor(0,55-18);
 display.print("Set Temp: ");
 display.print(settings.value[PCRstate*2]);
 display.print(" \xA7");display.print("C");

 display.setCursor(0,55-9);
 display.print("Block Temp: ");
 display.print(temperature_mean,1);
 display.print(" \xA7");display.print("C");
 display.setCursor(0,55);
 display.print("Time: ");
  display.print(TIMEcontrol);
 //display.print(" ");
   //display.print(TEMPcontrol);
// display.print(" ");
 // display.print(heatPower);
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
   // Serial.println(counter);
  } else if (result == DIR_CCW) {
    counter--;
   // Serial.println(counter);
  }
}

int power_heating(float temperature,float power) 
{
float power_return;

#define FA_c 0.00564542
#define FA_d 0.0418254
#define FA_e 0.000019826
#define FA_f 0.003711

power_return = (FA_d-FA_f*temperature-power)/(FA_e*temperature-FA_c);

if (power_return>255) power_return=255;
if (power_return<0) power_return=0;

return (int)power_return;

}
  
int power_cooling(float temperature,float power) 
{
float power_return;
  
#define FA_i 0.000072781
#define FA_j 0.00413579
#define FA_k 0.001372876
#define FA_l 0.1204656

power_return =(FA_l-FA_j*temperature-power)/(FA_i*temperature-FA_k);

if (power_return>255) power_return=255;
if (power_return<0) power_return=0;

return (int)power_return;
}

void runPID()
{
   TEMPcontrol= PIDp*TEMPdif+PIDd*TEMPd+(int)PIDIntegration*PIDi*TEMPi;
  setHeater(temperature_mean, TEMPcontrol);

  }

  void setHeater(float temperature,float power)
{
  if (power==0) {
    pinMode(fanPin, OUTPUT);
    pinMode(heaterPin, OUTPUT);
    digitalWrite(fanPin,LOW);
    digitalWrite(heaterPin,LOW);
  }
  else 
  {
  if (TEMPcontrol>0)
  {
    heatPower=power_heating(temperature,power);
  analogWrite(fanPin,0);
  analogWrite(heaterPin,heatPower);
  }
  else 
  {
    heatPower=power_cooling(temperature,power);
  analogWrite(fanPin,heatPower);
  analogWrite(heaterPin, 0);
    } 
  } // else
}
