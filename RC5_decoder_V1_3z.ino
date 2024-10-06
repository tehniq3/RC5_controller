/*
  RC5 decoder for audio selector + DC motor volume -> Nicu FLORICA (niq_ro) for Aurel IONESCU
  v.1.0 - initial version (base features)
  v.1.0.a - small corection to mute and channels
  v.1.1 - try to select uniq commands... still stuck
  v.1.2 - removed stack after first IR command
  v.1.2.a - added some delay for remotecontrol
  v.1.2.b - ignore IR remote when power is off
  v.1.2.c - swap D7 with D8 (for IR sensor)
  v.1.3 - added i2c LCD1602 
  v.1.3.z -  changed timmings for volume from IR commands 

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
 */

#include <Wire.h> 
#include <LiquidCrystal_I2C.h> 
#include <RC5.h>  // https://github.com/guyc/RC5/tree/master

#define SWPWR     2
#define SWMUTE    3
#define VOLminus  4
#define VOLplus   5
#define CHminus   6
#define CHplus    7
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

LiquidCrystal_I2C lcd(0x27, 16, 2);  // // Set the
// LCD address to 0c3F (or 0x27) for a 16 chars and 2 line display
RC5 rc5(IRpin);

byte starepwr  =  0;
byte starepwr0 =  0;
byte staremute =  0;
byte staremute0 = 0;
byte starevol   = 0;  // 1 = increase, 2 = decrease
byte starevol0  = 0;
int ch  =        3;  // 1 = min, 5 = max
int ch0 =        1;  

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

void setup() {                
  Serial.begin(9600);
  Serial.println("  ");
  Serial.println("Starting RC5 decoder v.1.3z");

  pinMode(SWPWR, INPUT);
  digitalWrite(SWPWR, HIGH); // activate pull-up resistor
  pinMode(SWMUTE, INPUT);
  digitalWrite(SWMUTE, HIGH); // activate pull-up resistor
  pinMode(VOLminus, INPUT);
  digitalWrite(VOLminus, HIGH); // activate pull-up resistor
  pinMode(VOLplus, INPUT);
  digitalWrite(VOLplus, HIGH); // activate pull-up resistor
  pinMode(CHminus, INPUT);
  digitalWrite(CHminus, HIGH); // activate pull-up resistor
  pinMode(CHplus, INPUT);
  digitalWrite(CHplus, HIGH); // activate pull-up resistor

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

  lcd.begin();  // initialize the LCD

  lcd.clear();  // clear the screen
  lcd.backlight();  // Turn on the blacklight
  lcd.setCursor(0,0); 
  lcd.print(" RC5 controller ");  
  lcd.setCursor(0,1);  
  lcd.print("v.1.3z by niq_ro"); 
  delay(3000);
  lcd.clear(); 

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
  // acum = 7;
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

//if (acum != 7)
if (millis() - tpchgvolum > tpvolum)
 {
  digitalWrite(MOT1, 0);
  digitalWrite(MOT2, 0);
 }
 
MUTEstate0 = MUTEreading; 
CHplusstate0 = CHplusreading;
CHminusstate0 = CHminusreading;
ch0 = ch;
}

if (starepwr%2 == 0)  // if power is OFF -> all off
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
//  lcd.noBacklight();  // Turn off the blacklight
//  lcd.clear();
  }
else
//if (PWRstate0 != PWRstate) 
  {
  digitalWrite(PWR, 1);
  digitalWrite(MUTE,staremute%2); 
//  digitalWrite(MOT1, 0);
//  digitalWrite(MOT2, 0);    
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
 // if (acum != starepwr%2)
 // lcd.backlight();  // Turn on the blacklight
 /*
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
    */ 
  } // end power on 

if (acum != starepwr%2)
{
  if (starepwr%2 == 0)
    {
    lcd.setCursor(0,0); 
    lcd.print("OFF             "); 
    lcd.setCursor(0,1); 
    lcd.print("                "); 
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
    }
}

PWRstate0 = PWRreading; 
acum = starepwr%2;
}  // end main loop
