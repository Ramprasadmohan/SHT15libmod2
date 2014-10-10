// OSBSS SHT15 library
// Based on code from http://bildr.org/2012/11/sht15-arduino/

// 9/2/2014 - Added temperature parameter to getHumidity function.
// This temperature is now being read from the thermistor instead of the SHT15

#include "Arduino.h"
#include "SHT15libmod2.h"

SHT15::SHT15(int SHT_clockPin, int SHT_dataPin)
{
  _SHT_dataPin = SHT_dataPin;
  _SHT_clockPin = SHT_clockPin;
}

float SHT15::getTemperature(){
  //Return Temperature in Celsius
  SHT15::SHT_sendCommand(B00000011, _SHT_dataPin, _SHT_clockPin);
  SHT15::SHT_waitForResult(_SHT_dataPin);

  int val = SHT_getData(_SHT_dataPin, _SHT_clockPin);
  SHT15::SHT_skipCrc(_SHT_dataPin, _SHT_clockPin);
  float temp_F = (-39.65)+(0.018*val);  // use 39.65 for 3.3V, 40.1 for 5V, 0.01 for C, 0.018 for F
	float temp_C = (temp_F-32)*0.5556;
  return temp_C; 
}

float SHT15::getHumidity(float T){
  //Return  Relative Humidity
  SHT_sendCommand(B00000101, _SHT_dataPin, _SHT_clockPin);
  SHT_waitForResult(_SHT_dataPin);
  int val = SHT_getData(_SHT_dataPin, _SHT_clockPin);
  SHT_skipCrc(_SHT_dataPin, _SHT_clockPin);
  float trueRH = (-2.0468) + (0.0367 * val) + (-0.0000015955 * val * val);
  float compensatedRH = (T-25)*(0.01+(0.00008*val))+trueRH;
  return compensatedRH;
}

float SHT15::getDewPoint(float T, float RH){
  //Return Dew Point
  float Tn,m;
	float temp_C = T;
  if(temp_C>0)
  {
	Tn = 243.12;
	m = 17.62;
  }
  if(temp_C<0)
  {
	Tn = 272.62;
	m = 22.46;
  }
  float dewPoint = Tn * (((log(RH/100))+((m*temp_C)/(Tn+temp_C)))/(m-(log(RH/100))-((m*temp_C)/(Tn+temp_C))));
  return dewPoint;
}


void SHT15::SHT_sendCommand(int command, int dataPin, int clockPin){
  // send a command to the SHTx sensor
  // transmission start
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, LOW);
  digitalWrite(dataPin, LOW);

  // shift out the command (the 3 MSB are address and must be 000, the last 5 bits are the command)
  shiftOut(dataPin, clockPin, MSBFIRST, command);

  // verify we get the right ACK
  digitalWrite(clockPin, HIGH);
  pinMode(dataPin, INPUT);

  if (digitalRead(dataPin)) Serial.println("ACK error 0");
  digitalWrite(clockPin, LOW);
  if (!digitalRead(dataPin)) Serial.println("ACK error 1");
}


void SHT15::SHT_waitForResult(int dataPin){
  // wait for the SHTx answer
  pinMode(dataPin, INPUT);

  int ack; //acknowledgement

  //need to wait up to 2 seconds for the value
  for (int i = 0; i < 1000; ++i){
    delay(2);
    ack = digitalRead(dataPin);
    if (ack == LOW) break;
  }

  if (ack == HIGH) Serial.println("ACK error 2");
}

int SHT15::SHT_getData(int dataPin, int clockPin){
  // get data from the SHTx sensor

  // get the MSB (most significant bits)
  pinMode(dataPin, INPUT);
  pinMode(clockPin, OUTPUT);
  byte MSB = shiftIn(dataPin, clockPin, MSBFIRST);

  // send the required ACK
  pinMode(dataPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);

  // get the LSB (less significant bits)
  pinMode(dataPin, INPUT);
  byte LSB = shiftIn(dataPin, clockPin, MSBFIRST);
  return ((MSB << 8) | LSB); //combine bits
}

void SHT15::SHT_skipCrc(int dataPin, int clockPin){
  // skip CRC data from the SHTx sensor
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  digitalWrite(dataPin, HIGH);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
}
