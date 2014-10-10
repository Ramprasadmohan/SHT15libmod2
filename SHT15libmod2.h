/*
  SHT15lib- library for getting temperature and humidity data from Sensirion's SHT15 temperature and
  humidity sensor.  Source code directly from BLDR blog, "sensing humidity with the sht15 + arduino"
*/

#ifndef SHT15libmod2_h
#define SHT15libmod2_h

#include "Arduino.h"

class SHT15
{
  public:
    SHT15(int SHT_clockPin, int SHT_dataPin);
    //void SHT15getTandRH(float &T, float &RH);
	float getTemperature();
    float getHumidity(float T);
	float getDewPoint(float T, float RH);
	void SHT_sendCommand(int command, int dataPin, int clockPin);
    void SHT_waitForResult(int dataPin);
    int SHT_getData(int dataPin, int clockPin);
    void SHT_skipCrc(int dataPin, int clockPin);
	
  private:
    int _SHT_clockPin;
    int _SHT_dataPin;
		//float temp_F;
		//float temp_C;
    //float getTemperature();
    //float getHumidity();
    
};

#endif


