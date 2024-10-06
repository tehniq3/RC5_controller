/*
  RC5 decoder for audio selector + DC motor volume -> Nicu FLORICA (niq_ro) for Aurel IONESCU (aureli67)
  v.1.0 - initial version (base features)
  v.1.0.a - small corection to mute and channels
  v.1.1 - try to select uniq commands... still stuck
  v.1.2 - removed stack after first IR command
  v.1.2.a - added some delay for remotecontrol
  v.1.2.b - ignore IR remote when power is off
  v.1.2.c - swap D7 with D8 (for IR sensor)
  v.1.3 - added i2c LCD1602 
  v.1.3.z -  changed timmings for volume from IR commands 
  v.1.4 - added one digit 7-segment display - https://github.com/tehniq3/PCF8574_leds
  v.1.5 - added new PCF8574 module with 2 buttons and 2 troggle outputs + remote
  v.1.5a - added new 2 buttons and 2 troggle outputs at last PCF8574 + remote
  v.1.5c - clean the sketch + added options (set type of digit + PCF outputs)
  v.1.6 - added other PCF8574 module with 4 buttons and 4 follow outputs
  v.1.6a - solved issue for long push on last pcf8574 (push -> output)
  v.1.7 - added encoder for channel switch control
  v.1.7a - decreased errors to encoder
  v.1.8 - solved issue with encoder (moved ins to 2 and 3)
  

Power        : Adress Sys 0x0 / 0 Command 0x0C / 12 
Mute         : Adress Sys 0x0 / 0 Command 0x0D / 13
Volume Up    : Adress Sys 0x0 / 0 Command 0x10 / 16
Volume Down  : Adress Sys 0x0 / 0 Command 0x11 / 17
Channel Up   : Adress Sys 0x0 / 0 Command 0x20 / 32
Channel Down : Adress Sys 0x0 / 0 Command 0x21 / 33
Channel 1    : Adress Sys 0x0 / 0 Command 0x01 / 1
Channel 2    : Adress Sys 0x0 / 0 Command 0x02 / 2
Channel 3    : Adress Sys 0x0 / 0 Command 0x03 / 3
Channel 4    : Adress Sys 0x0 / 0 Command 0x04 / 4
Channel 5    : Adress Sys 0x0 / 0 Command 0x05 / 5
extra commands:
TXT Red      :  0x55
TXT Green    :  0x54
TXT Yellow   :  0x50
TXT Blue     :  0x52
 */

#include "Arduino.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h> 
#include <RC5.h>  // https://github.com/guyc/RC5/tree/master
#include "PCF8574.h"  // https://github.com/xreef/PCF8574_library
#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h> // from http://www.pjrc.com/teensy/td_libs_Encoder.html


#define SWPWR     6 //2
#define SWMUTE    7 //3
#define VOLminus  4
#define VOLplus   5
#define CHminus   2 //6
#define CHplus    3 //7
#define IRpin     8
#define MOT1      9
#define MOT2     10
#define CH1      11
#define CH2      12
#define CH3      14  // A0
#define CH4      15  // A1
#define CH5      16  // A2
#define PWR      13
#define MUTE     17  // A3
#define sw1      0  // P0 - 1st pcf8574 module wih buttons and outputs
#define sw2      1  // P1
#define sw3      2  // P2    
#define sw4      3  // P3
#define led1     4  // P4
#define led2     5  // P5
#define led3     6  // P6
#define led4     7  // P7
#define sw11     0  // P0 - 2nd pcf8574 module wih buttons and outputs   
#define sw12     1  // P1
#define sw13     2  // P2    
#define sw14     3  // P3
#define led11    4  // P4
#define led12    5  // P5
#define led13    6  // P6
#define led14    7  // P7

#define afisajca  0  // 0 = common cathode, 1 = common anode
#define pcfsink   1  // 0 = output to gnd, 1 = outputs to plus (+5V)
#define remoteniq 1  // 1 - Nicu's remote, 0 - Aurel's remote

#define adresa   0x26 // adress for led display with i2c control
#define adresa2  0x24 // adress for 1st module with 4 buttons + 4 outputs (troggle outputs)
#define adresa3  0x25 // adress for 2nd module with 4 buttons + 4 outputs (momentary outputs)

LiquidCrystal_I2C lcd(0x27, 16, 2);  // // Set the
// LCD address to 0c3F (or 0x27) for a 16 chars and 2 line display
RC5 rc5(IRpin);

Encoder knob(CHminus, CHplus); //encoder connected (and ground)

PCF8574 pcf8574(adresa2); // original 0x27
PCF8574 pcf8574a(adresa3); // original 0x27

byte starepwr  =  0;
byte starepwr0 =  0;
byte staremute =  0;
byte staremute0 = 0;
byte starevol   = 0;  // 1 = increase, 2 = decrease
byte starevol0  = 0;
int ch  =        3;  // 1 = min, 5 = max
int ch0 =        0;  

// Variables will change:
int PWRstate;          // the current reading from the input pin
int PWRstate0 = 1;     // the previous reading from the input pin
int MUTEstate;         // the current reading from the input pin
int MUTEstate0 = 1;    // the previous reading from the input pin
int CHplusstate;       // the current reading from the input pin
int CHplusstate0 = 1 ; // the previous reading from the input pin
int CHminusstate;       // the current reading from the input pin
int CHminusstate0 = 1;    // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long PWRtime = 0;  // the last time the output pin was toggled
unsigned long MUTEtime = 0;  // the last time the output pin was toggled
unsigned long CHplustime = 0;  // the last time the output pin was toggled
unsigned long CHminustime = 0;  // the last time the output pin was toggled

unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
int intarziere = 250;
int acum = 7;
unsigned long tpchgvolum;
unsigned long tpvolum = 250;

byte staresw1 = 1 - pcfsink;  // intial state of 1st output
byte staresw2 = 1 - pcfsink;  // intial state of 2nd output
byte staresw3 = 1 - pcfsink;  // intial state of 3rd output
byte staresw4 = 1 - pcfsink;  // intial state of 4th output
unsigned long SWtime1 = 0;  // the time of the push the 1st switch 
unsigned long SWtime2 = 0;  // the time of the push the 2nd switch
unsigned long SWtime3 = 0;  // the time of the push the 3rd switch 
unsigned long SWtime4 = 0;  // the time of the push the 4th switch
int pauzapcf = 250;
unsigned long PCFdelay1 = 1500; // ignore time for push the buttons for troggle outputs
unsigned long PCFdelay2 =  250; // ignore time for push the buttons for momentary outputs

byte txt = 0;  // 
unsigned long tpoprire;

byte staresw11 = 1 - pcfsink;  // intial state of 1st output
byte staresw12 = 1 - pcfsink;  // intial state of 2nd output
byte staresw13 = 1 - pcfsink;  // intial state of 3rd output
byte staresw14 = 1 - pcfsink;  // intial state of 4th output
unsigned long SWtime11 = 0;  // the time of the push the 1st switch 
unsigned long SWtime12 = 0;  // the time of the push the 2nd switch
unsigned long SWtime13 = 0;  // the time of the push the 3rd switch 
unsigned long SWtime14 = 0;  // the time of the push the 4th switch
byte txt2 = 1;
unsigned long tptxt2;
int pozitie = -999;
unsigned long tpencoder = 0;

long oldPosition  = -999;
int knobval; // value for the rotation of the knob

void setup() {                
  Serial.begin(9600);
  Serial.println("  ");
  Serial.println("Starting RC5 decoder v.1.8  ");
  Wire.begin();

  pinMode(SWPWR, INPUT);
  digitalWrite(SWPWR, HIGH); // activate pull-up resistor
  pinMode(SWMUTE, INPUT);
  digitalWrite(SWMUTE, HIGH); // activate pull-up resistor
  pinMode(VOLminus, INPUT);
  digitalWrite(VOLminus, HIGH); // activate pull-up resistor
  pinMode(VOLplus, INPUT);
  digitalWrite(VOLplus, HIGH); // activate pull-up resistor
//  pinMode(CHminus, INPUT);
//  digitalWrite(CHminus, HIGH); // activate pull-up resistor
//  pinMode(CHplus, INPUT);
//  digitalWrite(CHplus, HIGH); // activate pull-up resistor

  pinMode(MOT1, OUTPUT);
  digitalWrite(MOT1, 0);   
  pinMode(MOT2, OUTPUT);
  digitalWrite(MOT2, 0);     
  pinMode(CH1, OUTPUT);
  digitalWrite(CH1, 0);   
  pinMode(CH2, OUTPUT);
  digitalWrite(CH2, 0);   
  pinMode(CH3, OUTPUT);
  digitalWrite(CH3, 0);   
  pinMode(CH4, OUTPUT);
  digitalWrite(CH4, 0);
  pinMode(CH5, OUTPUT);
  digitalWrite(CH5, 0);   
  pinMode(PWR, OUTPUT);
  digitalWrite(PWR, 0);
  pinMode(MUTE, OUTPUT);
  digitalWrite(MUTE, 0);   

  pcf8574.pinMode(sw1, INPUT_PULLUP);
  pcf8574.pinMode(sw2, INPUT_PULLUP);
  pcf8574.pinMode(sw3, INPUT_PULLUP);
  pcf8574.pinMode(sw4, INPUT_PULLUP); 
  pcf8574a.pinMode(sw11, INPUT_PULLUP);
  pcf8574a.pinMode(sw12, INPUT_PULLUP);
  pcf8574a.pinMode(sw13, INPUT_PULLUP);
  pcf8574a.pinMode(sw14, INPUT_PULLUP); 
  pcf8574.pinMode(led1, OUTPUT);
  pcf8574.pinMode(led2, OUTPUT);
  pcf8574.pinMode(led3, OUTPUT);
  pcf8574.pinMode(led4, OUTPUT);
  pcf8574a.pinMode(led11, OUTPUT);
  pcf8574a.pinMode(led12, OUTPUT);
  pcf8574a.pinMode(led13, OUTPUT);
  pcf8574a.pinMode(led14, OUTPUT);
  
  if(pcf8574.begin())
    Serial.println("OK");
    else 
    Serial.println("Failed");

  if(pcf8574a.begin())
    Serial.println("OK");
    else 
    Serial.println("Failed");
    
  lcd.begin();  // initialize the LCD

  lcd.clear();  // clear the screen
  lcd.backlight();  // Turn on the blacklight
  lcd.setCursor(0,0); 
  lcd.print(" RC5 controller ");  
  lcd.setCursor(0,1);  
  lcd.print("v.1.8 by niq_ro "); 
  delay(3000);
  lcd.clear(); 
  pcf8574.digitalWrite(led1, pcfsink);
  pcf8574.digitalWrite(led2, pcfsink);
  pcf8574.digitalWrite(led3, pcfsink);
  pcf8574.digitalWrite(led4, pcfsink);
  pcf8574a.digitalWrite(led11, pcfsink);
  pcf8574a.digitalWrite(led12, pcfsink);
  pcf8574a.digitalWrite(led13, pcfsink);
  pcf8574a.digitalWrite(led14, pcfsink); 
  knob.write(0);  // reset encoder
}

void loop() {
  
  unsigned char toggle;
  unsigned char address;
  unsigned char command;
  if (rc5.read(&toggle, &address, &command))  // if a signal is receved and decoded
  {
    Serial.print("a:");
    Serial.print(address);
    Serial.print(" c:");
    Serial.print(command);
    Serial.print(" t:");
    Serial.println(toggle);
    acum = 7;
  }
  else
    command = 99;

   if (command == 12)  // power
   {
    starepwr = starepwr + 1;
    tpoprire = millis();
    if (starepwr%2 == 0)  
      Serial.println("Power OFF");
    else
      Serial.println("Power ON"); 
    delay(intarziere);
   }

   if (starepwr%2 == 1)
   {
   if (command == 13)  // mute
   {
    staremute = staremute + 1;
    if (staremute%2 == 1)  
      Serial.println("MUTE");
    else
      Serial.println("unMUTE"); 
    delay(intarziere);
   }
   if (command == 16)  // volume +
   {
    starevol = 1;
    Serial.println("VOL +"); 
    digitalWrite(MOT1, 1);
    digitalWrite(MOT2, 0);
    //delay(3*intarziere);
    tpchgvolum = millis();
   }
   if (command == 17)  // volume -
   {
    starevol = 2;
    Serial.println("VOL -");
    digitalWrite(MOT1, 0);
    digitalWrite(MOT2, 1);
    //delay(3*intarziere);
    tpchgvolum = millis();
   }
   if (command == 32)  // ch +
   {
    ch = ch + 1;
    Serial.println("CH +");
    if (ch > 5)
    {
      ch = 5;
      ch0 = 5;
    }
       Serial.print("CH = ");
       Serial.println(ch);
       delay(intarziere);
   }
   if (command == 33)  // ch -
   {
    ch = ch - 1;
    Serial.println("CH -");
    if (ch < 1) 
    {
      ch = 1;
      ch0 = 1;
    }
       Serial.print("CH = ");
       Serial.println(ch);
       delay(intarziere);
   }
   if (command == 1)  // ch = 1
   {
    ch = 1;
    Serial.println("CH = 1");
   }
   if (command == 2)  // ch = 2
   {
    ch = 2;
    Serial.println("CH = 2");
   }
   if (command == 3)  // ch = 3
   {
    ch = 3;
    Serial.println("CH = 3");
   }
   if (command == 4)  // ch = 4
   {
    ch = 4;
    Serial.println("CH = 4");
   }
   if (command == 5)  // ch = 5
   {
    ch = 5;
    Serial.println("CH = 5");
   }
  
      if (command == 55)  // red txt
   {
    staresw1 = staresw1 + 1;
    if (staresw1%2 == 1)  
      Serial.println("TXT SW1 on");
    else
      Serial.println("TXT SW1 off"); 
    delay(intarziere);
    txt = 1;
   }
      if (command == 54)  // green txt
   {
    staresw2 = staresw2 + 1;
    if (staresw2%2 == 1)  
      Serial.println("TXT SW2 on");
    else
      Serial.println("TXT SW2 off"); 
    delay(intarziere);
    txt = 1;
   }
      if (command == 50)  // yellow txt
   {
    staresw3 = staresw3 + 1;
    if (staresw3%2 == 1)  
      Serial.println("TXT SW3 on");
    else
      Serial.println("TXT SW3 off"); 
    delay(intarziere);
    txt = 1;
   }
      if (command == 52)  // blue txt
   {
    staresw4 = staresw4 + 1;
    if (staresw4%2 == 1)  
      Serial.println("TXT SW4 on");
    else
      Serial.println("TXT SW4 off"); 
    delay(intarziere);
    txt = 1;
   }

      if (command == 20)  // extra left button (1st)
   {
    staresw11 = 1;
      Serial.println("extra SW11 was push");
    txt2 = 1;
   }
      if (command == 41)  // extra 2nd button
   {
    staresw12 = 1; 
      Serial.println("extra SW12 was push");
    txt2 = 1;
   }
      if (command == 44)  // extra 3rd button
   {
    staresw13 = 1; 
      Serial.println("extra SW13 was push");
    txt2 = 1;
   }
      if (command == 21)  // extra 4th button
   {
    staresw14 = 1; 
      Serial.println("extra SW14 was push");
    txt2 = 1;
   }

   } // receive IR control just if power is on

// read local buttons
  int PWRreading = digitalRead(SWPWR);  // read the state of the switch into a local variable:
  if (PWRreading != PWRstate0)   // If the switch changed, due to noise or pressing:
    {  
    PWRtime = millis();  // reset the debouncing timer
    }
  if ((millis() - PWRtime) > debounceDelay) 
    { 
    if (PWRreading != PWRstate)  // if the button state has changed:
      {
      PWRstate = PWRreading;    
      if (PWRstate == LOW)  // only toggle the power state, if the new button state is LOW (push)
      {
       starepwr = starepwr + 1;
       tpoprire = millis();
       if (starepwr%2 == 0)  
         Serial.println("Power OFF");
       else
         Serial.println("Power ON"); 
      }
      }
    }

if (starepwr%2 != 0)  // local commands just if device is ON
{
  int MUTEreading = digitalRead(SWMUTE);  // read the state of the switch into a local variable:
  if (MUTEreading != MUTEstate0)   // If the switch changed, due to noise or pressing:
    {  
    MUTEtime = millis();  // reset the debouncing timer
    }
  if ((millis() - MUTEtime) > debounceDelay) 
    { 
    if (MUTEreading != MUTEstate)  // if the button state has changed:
      {
      MUTEstate = MUTEreading;    
      if (MUTEstate == LOW)  // only toggle the power state, if the new button state is LOW (push)
      {
        staremute = staremute + 1;
        acum = 7;
        if (staremute%2 == 0)  
         Serial.println("unMUTE");
       else
         Serial.println("MUTE"); 
      }
      }
    }

/*
  int CHminusreading = digitalRead(CHminus);  // read the state of the switch into a local variable:
  if (CHminusreading != CHminusstate0)   // If the switch changed, due to noise or pressing:
    {  
    CHminustime = millis();  // reset the debouncing timer
    }
  if ((millis() - CHminustime) > debounceDelay) 
    { 
    if (CHminusreading != CHminusstate)  // if the button state has changed:
      {
      CHminusstate = CHminusreading;    
      if (CHminusstate == LOW)  // only toggle the power state, if the new button state is LOW (push)
      {
        ch = ch - 1;
        acum = 7;
        Serial.println("CH -");
       if (ch < 1) 
       {
        ch = 1;
        ch0 = 1;
       } 
       Serial.print("CH = ");
       Serial.println(ch);
      }
      }
    }
*/

/*
  int CHplusreading = digitalRead(CHplus);  // read the state of the switch into a local variable:
  if (CHplusreading != CHplusstate0)   // If the switch changed, due to noise or pressing:
    {  
    CHplustime = millis();  // reset the debouncing timer
    }
  if ((millis() - CHplustime) > debounceDelay) 
    { 
    if (CHplusreading != CHplusstate)  // if the button state has changed:
      {
      CHplusstate = CHplusreading;    
      if (CHplusstate == LOW)  // only toggle the power state, if the new button state is LOW (push)
      {
        ch = ch + 1;
        acum = 7;
        Serial.println("CH +");
       if (ch > 5) 
       {
        ch = 5;
        ch0 = 5;
       } 
       Serial.print("CH = ");
       Serial.println(ch);
      }
      }
    }
*/
    
 //   knob.write(0);
  //  delay (50);
    knobval=knob.read();
    if (knobval < -1) 
    { //bit of software de-bounce
      knobval = -1;
      Serial.println(knobval);
      knob.write(0);
      delay (50);
             pozitie = 1;
     ch = ch - 1;
     acum = 7;
     Serial.println("CH -");
     if (ch < 1) 
       {
        ch = 1;
        ch0 = 1;
       } 
       Serial.print("CH = ");
       Serial.println(ch);
       
    }
    if (knobval > 1) 
    {
      knobval = 1;
      Serial.println(knobval);
      knob.write(0);
      delay (50);
      pozitie = 2;  
       ch = ch + 1;
       acum = 7;
       Serial.println("CH +");
       if (ch > 5) 
        {
        ch = 5;
        ch0 = 5;
        } 
       Serial.print("CH = ");
       Serial.println(ch);
    }
   
   /* 
  knob.write(0);
  long newPosition = knob.read();
  if (newPosition != oldPosition) 
  {
  tpencoder = millis();
  if (tpencoder - millis() > PCFdelay1)
  {
  if (newPosition > oldPosition)
    {
    pozitie = 2;  
     ch = ch + 1;
     acum = 7;
     Serial.println("CH +");
     if (ch > 5) 
       {
        ch = 5;
        ch0 = 5;
       } 
       Serial.print("CH = ");
       Serial.println(ch);
      delay(tpvolum); 
      //delay(PCFdelay1);
    }
  else
  if (newPosition < oldPosition)
  {
    pozitie = 1;
     ch = ch - 1;
     acum = 7;
     Serial.println("CH -");
     if (ch < 1) 
       {
        ch = 1;
        ch0 = 1;
       } 
       Serial.print("CH = ");
       Serial.println(ch);
       delay(tpvolum); 
     //  delay(PCFdelay1);
  }
  else
    pozitie = 0;
    oldPosition = newPosition;
    Serial.println(newPosition);
    Serial.print(newPosition);
    Serial.print("/");
    Serial.println(pozitie);
    delay(tpvolum); 
  //delay(PCFdelay1);
  }
  }  
*/ 
/*
 if (newPosition > oldPosition)
    {
     ch = ch + 1;
     acum = 7;
     Serial.println("CH +");
     if (ch > 5) 
       {
        ch = 5;
        ch0 = 5;
       } 
       Serial.print("CH = ");
       Serial.println(ch);
       delay(tpvolum); 
      }
 if (newPosition < oldPosition)
    {
     ch = ch - 1;
     acum = 7;
     Serial.println("CH -");
     if (ch < 1) 
       {
        ch = 1;
        ch0 =1;
       } 
       Serial.print("CH = ");
       Serial.println(ch);
       delay(tpvolum); 
    }
      // delay(tpvolum);  
    oldPosition = newPosition;
    delay(tpvolum);   
   // encoder changed

*/
if (digitalRead(VOLplus) == LOW)
 {
  Serial.println("VOL +");
  digitalWrite(MOT1, 1);
  digitalWrite(MOT2, 0);
  tpchgvolum = millis();
 }

if (digitalRead(VOLminus) == LOW)
 {
  Serial.println("VOL -");
  digitalWrite(MOT1, 0);
  digitalWrite(MOT2, 1);
  tpchgvolum = millis();
 }

int SWreading1 = pcf8574.digitalRead(sw1);  // read the state of the switch into a local variable:
int SWreading2 = pcf8574.digitalRead(sw2);  // read the state of the switch into a local variable:
int SWreading3 = pcf8574.digitalRead(sw3);  // read the state of the switch into a local variable:
int SWreading4 = 1; // sw3 deactivated on standard i2c module for alphanumerical display (1602 or 2004)
// int SWreading4 = pcf8574.digitalRead(sw4);  // read the state of the switch into a local variable:

int SWreading11 = pcf8574a.digitalRead(sw11);  // read the state of the switch into a local variable:
int SWreading12 = pcf8574a.digitalRead(sw12);  // read the state of the switch into a local variable:
int SWreading13 = pcf8574a.digitalRead(sw13);  // read the state of the switch into a local variable:
int SWreading14 = 1; // sw3 deactivated on standard i2c module for alphanumerical display (1602 or 2004)
// int SWreading14 = pcf8574a.digitalRead(sw14);  // read the state of the switch into a local variable:
 
if ((SWreading1 == 0) and (millis() - SWtime1 > PCFdelay1))       
  {
     staresw1 = staresw1 + 1;
     Serial.println("TXT SW1 was pressed");
     SWtime1 = millis();  // reset the debouncing timer
     txt = 1;
     delay(pauzapcf);
  }

  if ((SWreading2 == 0) and (millis() - SWtime2 > PCFdelay1))
  {
     staresw2 = staresw2 + 1;
     Serial.println("TXT SW2 was pressed");
     SWtime2 = millis();  // reset the debouncing timer
     txt = 1;
     delay(pauzapcf);
  }

  if ((SWreading3 == 0) and (millis() - SWtime3 > PCFdelay1))
  {
     staresw3 = staresw3 + 1;
     Serial.println("TXT SW3 was pressed");
     SWtime3 = millis();  // reset the debouncing timer
     delay(pauzapcf);
     txt = 1;
  }

  if ((SWreading4 == 0) and (millis() - SWtime4 > PCFdelay1))
  {
     staresw4 = staresw4 + 1;
     Serial.println("TXT SW4 was pressed");
     SWtime4 = millis();  // reset the debouncing timer
     txt = 1;
     delay(pauzapcf);
  }

if ((SWreading11 == 0) and (millis() - SWtime11 > PCFdelay2))       
  {
     staresw11 = 1;
     Serial.println("extra SW11 was pressed");
     SWtime11 = millis();  // reset the debouncing timer
     txt2 = 1;
   //  delay(pauzapcf);
  }

if ((SWreading12 == 0) and (millis() - SWtime12 > PCFdelay2))       
  {
     staresw12 = 1;
     Serial.println("extra SW12 was pressed");
     SWtime12 = millis();  // reset the debouncing timer
     txt2 = 1;
   //  delay(pauzapcf);
  }

if ((SWreading13 == 0) and (millis() - SWtime13 > PCFdelay2))       
  {
     staresw13 = 1;
     Serial.println("extra SW13 was pressed");
     SWtime13 = millis();  // reset the debouncing timer
     txt2 = 1;
   //  delay(pauzapcf);
  }

if ((SWreading14 == 0) and (millis() - SWtime14 > PCFdelay2))       
  {
     staresw14 = 1;
     Serial.println("extra SW14 was pressed");
     SWtime14 = millis();  // reset the debouncing timer
     txt2 = 1;
   //  delay(pauzapcf);
  }

if (millis() - tpchgvolum > tpvolum)
 {
  digitalWrite(MOT1, 0);
  digitalWrite(MOT2, 0);
 }

MUTEstate0 = MUTEreading; 
//CHplusstate0 = CHplusreading;
//CHminusstate0 = CHminusreading;
ch0 = ch;
}

if (starepwr%2 == 0)  // if power is OFF -> all off
//if ((PWRstate0 != PWRstate) and (starepwr%2 == 0))
  {
  digitalWrite(PWR, 0);
  digitalWrite(MUTE,0);     
  digitalWrite(CH1, 0);   
  digitalWrite(CH2, 0);   
  digitalWrite(CH3, 0);   
  digitalWrite(CH4, 0);
  digitalWrite(CH5, 0); 
  digitalWrite(MOT1, 0);
  digitalWrite(MOT2, 0);
  }
if (starepwr%2 == 1)
  {
  digitalWrite(PWR, 1);
  digitalWrite(MUTE,staremute%2);     
  digitalWrite(CH1, 0);   
  digitalWrite(CH2, 0);   
  digitalWrite(CH3, 0);   
  digitalWrite(CH4, 0);
  digitalWrite(CH5, 0);
  if (ch == 1)
     digitalWrite(CH1, 1);  
  if (ch == 2)
     digitalWrite(CH2, 1); 
  if (ch == 3)
     digitalWrite(CH3, 1); 
  if (ch == 4)
     digitalWrite(CH4, 1);        
  if (ch == 5)
     digitalWrite(CH5, 1);  
  } // end power on 

if ((txt == 2) and (starepwr%2 == 1))
{
 txt = 1;  
}


if (acum != starepwr%2)
{
  if (starepwr%2 == 0)
    {
    lcd.setCursor(0,0); 
    lcd.print("OFF             "); 
    lcd.setCursor(0,1); 
    lcd.print("                "); 
    Wire.beginTransmission(adresa); // transmit to PCF8574
    if (afisajca == 0)
    Wire.write(0x00);              // sends 9 to 7-Segment
    else
    Wire.write(0xFF);
    Wire.endTransmission();    // stop transmitting
    txt = 2; 
    }
  else
    {
    lcd.setCursor(0,0);
    lcd.print("ON "); 
    lcd.setCursor(0,1); 
    lcd.print("CH:");
    lcd.print(ch);  
    lcd.setCursor(0,1); 
    lcd.print("CH:");
    lcd.print(ch);
    lcd.setCursor(12,0); 
    lcd.print("    ");
    lcd.setCursor(12,1);
    if (staremute%2 == 0)
      lcd.print("    ");
     else
      lcd.print("MUTE");
      Wire.beginTransmission(adresa); // transmit to PCF8574
      if (ch == 0)
         if (afisajca == 0)
         Wire.write(0x77);
         else
         Wire.write(0x80);
      if (ch == 1)
         if (afisajca == 0)
         Wire.write(0x06);
         else
         Wire.write(0xf1);
      if (ch == 2)
         if (afisajca == 0)
         Wire.write(0xb3);
         else
         Wire.write(0x44);
      if (ch == 3)
         if (afisajca == 0)
         Wire.write(0x97);
         else
         Wire.write(0x60);
      if (ch == 4)
         if (afisajca == 0)
         Wire.write(0xc6);
         else
         Wire.write(0x31);
      if (ch == 5)
         if (afisajca == 0)
         Wire.write(0xd5);
         else
         Wire.write(0x22);       
      Wire.endTransmission();    // stop transmitting
    }
}

if ((txt == 1) and (starepwr%2 == 1))
 { 
  pcf8574.digitalWrite(led1, (pcfsink + staresw1)%2);
  pcf8574.digitalWrite(led2, (pcfsink + staresw2)%2);
  pcf8574.digitalWrite(led3, (pcfsink + staresw3)%2);
  pcf8574.digitalWrite(led4, (pcfsink + staresw4)%2);
  txt = 0;
  }

if ((txt2 == 1) and (starepwr%2 == 1))
 {
  pcf8574a.digitalWrite(led11, (pcfsink + staresw11)%2);
  pcf8574a.digitalWrite(led12, (pcfsink + staresw12)%2);
  pcf8574a.digitalWrite(led13, (pcfsink + staresw13)%2);
  pcf8574a.digitalWrite(led14, (pcfsink + staresw14)%2);
  tptxt2 = millis();
 // delay(pauzapcf);
  txt2 = 3;  
  }
 
if ((txt2 == 3) and (millis()-tptxt2 > pauzapcf))
 { 
  pcf8574a.digitalWrite(led11, (pcfsink)%2);
  pcf8574a.digitalWrite(led12, (pcfsink)%2);
  pcf8574a.digitalWrite(led13, (pcfsink)%2);
  pcf8574a.digitalWrite(led14, (pcfsink)%2);
  staresw11 = 0;
  staresw12 = 0;
  staresw13 = 0;
  staresw14 = 0;
//  delay(pauzapcf);
  txt2 = 0;
  }
 
if ((millis() - tpoprire < 1000) and (starepwr%2 == 0))
 { 
  pcf8574.digitalWrite(led1, pcfsink);
  pcf8574.digitalWrite(led2, pcfsink);
  pcf8574.digitalWrite(led3, pcfsink);
  pcf8574.digitalWrite(led4, pcfsink);
  txt = 1;
  }

PWRstate0 = PWRreading; 
acum = starepwr%2;
//  knob.write(0);  // reset encoder
}  // end main loop
