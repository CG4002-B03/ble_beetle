#include "TrueRMS.h"

#define EMG_INPUT_PIN 0     // define the used EMG input channel
#define RMS_WINDOW 50 // rms window of 40 samples

#define PIN_DEBUG 4

unsigned long nextLoop;
int adcVal;
int cnt=0;
float VoltRange = 5.00; // The full scale value is set to 5.00 Volts but can be changed when using an
                        // input scaling circuit in front of the ADC.

long int imu_last_read_time = 0; //when the IMU was last read by the Beetle
int IMU_SAMPLE_INTERVAL = 10; //20 millis between IMU read

Rms readRms; // create an instance of Rms.


// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);

  readRms.begin(VoltRange, RMS_WINDOW, ADC_10BIT, BLR_ON, CNT_SCAN);
  readRms.start(); //start measuring
}

// the loop routine runs over and over again forever:
void loop() {
    if (millis() > imu_last_read_time + IMU_SAMPLE_INTERVAL) {
      imu_last_read_time = millis();

      // read the input on analog pin 0:
      int EMGValue = analogRead(EMG_INPUT_PIN);
      Serial.println(EMGValue);
      readRms.update(EMGValue);
      // print out the value you read:
//      Serial.println(sensorValue);
//      delay(1);        // delay in between reads for stability
//      cnt++;
//      if(cnt >= 50) { // publish every 1s aka window is full
//        readRms.publish();
//        Serial.print(readRms.rmsVal,2);
//        Serial.print(", ");
//        Serial.println(readRms.dcBias);
//        cnt=0;
//        //readRms.start();  // Restart the acquisition after publishing if the mode is single scan.
//      }
    }
}
