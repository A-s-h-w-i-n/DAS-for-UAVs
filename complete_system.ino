//#include <SoftwareSerial.h>
#include <NeoSWSerial.h>
#include <Wire.h>
#include <MS5611.h>
#include<Servo.h>
#include<Kalman.h>
#include<TinyGPS++.h>

TinyGPSPlus gps;
NeoSWSerial ss(4,3);
Kalman kalman(0.6, 0.6, 1, 0);
Kalman kalman_speed(0.5, 0.5, 1, 0);
MS5611 ms5611;
Servo servo1, servo2;
float starttime, afterdrop, start, afterdrop_gps, p, temp=0, c1, c2, h=0;
char t;
int flag = -1;

double referencePressure=0;

void setup()
{
  Serial.begin(57600);
  ss.begin(9600);

  servo1.attach(5);
  servo2.attach(9);
  servo1.write(126);
  servo2.write(126);
  
  ms5611.begin();
  delay(1000);

  // Get reference pressure for relative altitude
   for(int i=0; i<4; i++)  
   { 
     referencePressure += ms5611.readPressure();
     delay(10);
   } 
  referencePressure = referencePressure/4;

   for(int i=0; i<4; i++)
   {
     temp += ms5611.readTemperature();
     delay(10);
   }
   temp = temp/4;

   digitalWrite(6, HIGH);
   pinMode(6, OUTPUT);

   starttime = millis();
}
int c=0;

void loop()
{ 
  if(flag == -1)
  {
    flag=0;
    delay(1000);
  }

  start = millis();
  while((millis() - start) <= 300)
   {
    if(Serial.available())
      {
        t = Serial.read();
        if(t=='d' || t=='D')
        { 
          Serial.print("t\n");
          hatch_open();
          afterdrop = millis();
          flag = 1;
        }
        else if(t=='o' || t=='O')
        {
          hatch_open();
        }
        else if (t=='c' || t=='C')
        {
          hatch_1_close();
          delay(700);
          hatch_2_close();
        }
        else if(t=='r'||t=='R')
        {
          digitalWrite(6, LOW);
        }
      } 
      
    while (ss.available())
    gps.encode(ss.read());    
}
  
   calcP(p);
   c1 = calcHeight(p, temp);
   calcP(p);
   c2 = calcHeight(p, temp);
   h = kalman.getFilteredValue((c1+c2)/2);
 
 Serial.print("h"); 
 Serial.println(h, 2);

 if(gps.location.isValid())
 {
    Serial.println(gps.location.lat(), 6);
 }
  else
  {
    Serial.println(0.0, 6);  
  }
 if(gps.location.isValid())
   { 
    Serial.println(gps.location.lng(), 6);
   }
  else
    {
    Serial.println(0.0, 6);
    } 
      
 if(gps.location.isValid())
    {
    Serial.println(gps.speed.kmph(), 2);
    }
  else
  {
    Serial.println(0.0, 2);  
  } 
//IF HATCH IS OPEN
 if(flag == 1 && millis()-afterdrop >= 1500)
 {
    hatch_1_close();
 }

if(flag == 1 && millis()-afterdrop >=2100)
 {
  hatch_2_close();
  flag = 0;
 }
}

void hatch_open()
{
  servo1.write(0);
  servo2.write(0);
}

void hatch_1_close()
{
  servo1.write(126);
}  

void hatch_2_close()
{
  servo2.write(126);
}

float calcHeight(float p, float t)
{
  float a = pow((referencePressure/p), 0.19022) - 1;
  float b = a*(t + 273.15);
  float c2 = b/0.0065; 
  return c2*3.28084;
}

void calcP(float &p)
{
  p = 0; 
  for(int i=0; i<4; i++)  
   { 
     p += ms5611.readPressure();
     delay(10);
   } 
      
  p = p/4;
}

