//This code is a modification of code written for the Seeeduino xiao by David Taylor and released under the 
//GNU general public license V3.0. The code was written using an Adafruit BMP280 sensor with I2C communication for temperature
//and pressure values used to calculate compensation.
//When the Adafruit BMP280 sensor would fail, the controller would fail in an undetermined failure.  Several other sensors and libraries were 
//tested and failed in a similar fashion.  
//This code was written to replace the BMP280 sensor with discrete analog and pressure sensors.
//An Adafruit TMP36 was chosen for the temperature sensor and a Honeywell HSC series sensor with a 0 to 1.6 bar range was chosen for the barometric pressure sensor.
//When the temperature or pressure readings are outside their normal range, a failover data point is substitued.
//Additionally rolling averages for the temperature and pressure readings are implemented to minimize the effect of noise.
//Much of the code used for the display was eliminated since I am relying on telemetry for those values.

// Define the analog pins that the TMP36's and pressure sensor are connected to
#define tempPin A4
#define pressurePin A5

//Variables for temperature rolling average
const int numReadingsT = 200; // Number of readings to average (e.g., 8)
int readings[numReadingsT]; // The array to store readings
int readIndexT = 0;         // Current index in the array
volatile unsigned long totalT;            // Sum of readings (use long for large sums)
// The calculated average temperature reading
//int averageT = 0 ;

//Variables for pressure rolling average
const int numReadingsP = 200; // Number of readings to average (e.g., 8)
int readingsP[numReadingsP]; // The array to store readings
int readIndexP = 0;         // Current index in the array
volatile unsigned long totalP ;            // Sum of readings (use long for large sums)
float avgP;

//Variables for solenoid rolling average
const int numReadingsS = 33; // Number of readings to average (e.g., 8)
int readingsS[numReadingsS]; // The array to store readings
int readIndexS = 0;         // Current index in the array
volatile unsigned long totalS ;            // Sum of readings (use long for large sums)

unsigned long old_time;
//int update_display;
//const long display_interval = 1000; 
volatile unsigned long pulse_start_time = 0L;
volatile unsigned long pulse_now = 0L;
volatile unsigned long channel_pulse = 0L;
volatile unsigned long pulse_old = 0L;
volatile unsigned long adjusted_pulse = 0L;
volatile unsigned long fallback_corr = 0L;
volatile unsigned long solenoid_pulse = 0L;
volatile unsigned long usable_pulse = 0L;
int channel;

//Reference temp and pressure are the values while tuning engine degrees C and mb
const float reference_temp = 283.0;
const float reference_pressure = 980;

// Fallback temp and pressure replace actual measured values if the sensor fails during
// flight. Set fallback temp to the lowest temperature you're likely to fly in and the highest
// barometric pressure will cause the engine to run richer if fallback occurs.
const int fallback_temp = 10;
const float fallback_pressure = 1050;

const int rc_functionPin = 2;

#include <SPort.h> 
//#include <Wire.h>

int inPressure;
float temperatureCorrection;
float pressureCorrection;
float temperatureF;
float avgT;



SPortHub hub(0x12, 0);   
                 //Hardware ID 0x12, Software serial pin 0
CustomSPortSensor intake_temperature(getSensorData);    //Sensor with a callback function to get the data
CustomSPortSensor intake_pressure(getSensorData1);      //Sensor with a callback function to get the data
CustomSPortSensor compensation_ratio(getSensorData2);   //Sensor with a callback function to get the data
CustomSPortSensor solenoid_command(getSensorData3);     //Sensor with a callback function to get the data
CustomSPortSensor temperature_corr(getSensorData4);     //Sensor with a callback function to get the data
CustomSPortSensor pressure_corr(getSensorData5);        //Sensor with a callback function to get the data



void setup() {

  hub.registerSensor(intake_temperature);          //Add sensor to the hub
  hub.registerSensor(intake_pressure);             //Add sensor to the hub
  hub.registerSensor(compensation_ratio);          //Add sensor to the hub
  hub.registerSensor(solenoid_command);            //Add sensor to the hub
  hub.registerSensor(temperature_corr);            //Add sensor to the hub
  hub.registerSensor(pressure_corr);              //Add sensor to the hub


  hub.begin();                            //Start listening for s.port data
 

  // Begin serial communication at 9600 baud rate
  Serial.begin(9600);

//For temperature readings
  for (int thisReading = 0; thisReading < numReadingsT; thisReading++) {
    readings[thisReading] = 0;
  }

//For pressure readings
  for (int thisReading = 0; thisReading < numReadingsP; thisReading++) {
    readings[thisReading] = 0;
  }

  //For solenoid readings
  for (int thisReading = 0; thisReading < numReadingsS; thisReading++) {
    readings[thisReading] = 0;
  }

  pinMode(rc_functionPin, INPUT);
  attachInterrupt(rc_functionPin, RCpinread, CHANGE);

  //setup timer regisiters
  pinMode(3, OUTPUT);
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(232) |       // Divide the main clock down by some factor to get generic clock
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

// Enable the port multiplexer for the digital pin. 
  PORT->Group[g_APinDescription[3].ulPort].PINCFG[g_APinDescription[3].ulPin].bit.PMUXEN = 1;
  
//Connect the TCC0 timer to digital output - port pins are paired odd PMUO and even PMUXE 
  PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg = PORT_PMUX_PMUXO_E; // | PORT_PMUX_PMUXE_F;

// Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

//Set for Single slope PWM operation: timers or counters count up to TOP value and then repeat
  REG_TCC1_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        // Reverse the output polarity on all TCC0 outputs
  while (TCC1->SYNCBUSY.bit.WAVE);               // Wait for synchronization
 
  REG_TCC1_PER = 6144;                           // This sets the rate or frequency of PWM signal 
  while (TCC1->SYNCBUSY.bit.PER);                // Wait for synchronization
  
// Set the PWM signal to output 50% duty cycle initially 
  REG_TCC1_CC1 = 3072;                           // this sets the pwm duty cycle. values from 0 6145 are valid. 
  while (TCC1->SYNCBUSY.bit.CC1);                // Wait for synchronization

// Set prescaler and enable the outputs
  REG_TCC1_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}

void loop() {

   hub.handle();
    //Gather temperature readings
    // Subtract the oldest temp reading from the total
    totalT = totalT - readings[readIndexT];
      // Read the new temperature from TMP36
      float voltageT = analogRead(A4) * 3.3 / 1024.0;
      float temperatureC = (voltageT - 0.5) * 100.0;
        // Store the new reading in the array
        readings[readIndexT] = temperatureC;
           // Add the new reading to the total
           totalT = totalT + readings[readIndexT];
             // Advance the index to the next position
             readIndexT = readIndexT + 1;
                // If at the end of the array, wrap around to the beginning
                 if (readIndexT >= numReadingsT) {
                 readIndexT = 0;
  }
                  // Calculate the average
                  float avgT = totalT / numReadingsT;
                  //Convert float to int
                  //avgT = average ;

  //int press = analogRead(A5);
 
  // gather pressure preadings
  // Subtract the oldest pressurereading from the total
  totalP = totalP - readingsP[readIndexP];
    // Read the new pressure 
    float voltageP = analogRead(A5) * 3.3 / 1024.0;
    float bar = (((voltageP) - 0.33) / 1.65);
    float pressureHg = (bar * 1000 );
      // Store the new reading in the array
      readingsP[readIndexP] = pressureHg;
       // Add the new reading to the total
        totalP = totalP + readingsP[readIndexP];
          // Advance the index to the next position
           readIndexP = readIndexP + 1;
              // If we're at the end of the array, wrap around to the beginning
              if (readIndexP >= numReadingsP) {
              readIndexP = 0;
  }
                // Calculate the average
                float avgP = (totalP / numReadingsP);
                  
  //check and adjust for data quality
  if (avgT >= 85) avgT = fallback_temp;
  if (avgT <= 10) avgT = fallback_temp;
  if (avgP >= 1300) avgP = fallback_pressure; 
  if (avgP <= 800) avgP = fallback_pressure; 

            //Create data points for Sport data, I'm an old guy that likes British imperial system
             temperatureF = (avgT * 9.0 / 5.0) + 32.0;
             inPressure = ((avgP/10) *29.92);
             temperatureCorrection = (reference_temp/(avgT + 273));
             pressureCorrection = ((avgP/reference_pressure));

                //adjust pulse for temp and pressure  
                adjusted_pulse = ( usable_pulse * ((reference_temp/(avgT + 273))*(avgP/reference_pressure))) * 6.145;              
                solenoid_pulse = constrain(adjusted_pulse, 0, 6145);
                solenoid_pulse = map(solenoid_pulse, 0, 6145, 6145, 0);

                //Gather solenoid pulse reading for rolling average
                // gather pressure preadings
               // Subtract the oldest pressurereading from the total
                 totalS = totalS - readingsS[readIndexS];
                // Store the new reading in the array
                readingsS[readIndexS] = solenoid_pulse;
                // Add the new reading to the total
                totalS = totalS + readingsS[readIndexS];
                // Advance the index to the next position
                readIndexS = readIndexS + 1;
                // If we're at the end of the array, wrap around to the beginning
                if (readIndexS >= numReadingsS) {
                 readIndexS = 0;}
  
                // Calculate the average
                float avgS = (totalS / numReadingsS);

        // load pulse value into timer register
        REG_TCC1_CC1 = avgS; 
        while (TCC1->SYNCBUSY.bit.CC1);





// Uncommebt for debugging  
     
       //Serial.println(usable_pulse);
       //Serial.println(adjusted_pulse);
       //Serial.println(solenoid_pulse );
       // Serial.println( channel_pulse);
       //Serial.println(average);
      //Serial.println(avgT);
      // Serial.println(average);
       // Serial.println(readIndexP);
       //Serial.println(avgP);
       // Serial.println(numReadingsS);
      // Serial.println(temperatureCorrection);
       // Serial.println( pressureCorrection);
       //Serial.println( avgS);
       //Serial.println(pressureHg);
       //delay(500); // wait a second between readings
}

void RCpinread() {
  pulse_now = micros();
  pulse_old = channel_pulse;
  channel_pulse = pulse_now - pulse_start_time;
  pulse_start_time = pulse_now;
  if(channel_pulse < 988) {
    channel_pulse = pulse_old; //throw out low values
  }else if(channel_pulse > 2012) {
    channel_pulse = pulse_old; //throw out high values
  }
  channel_pulse = constrain (channel_pulse, 1000, 2000);
  channel = channel_pulse;
  usable_pulse = channel_pulse - 1000;
}

sportData getSensorData(CustomSPortSensor* intake_temperature) {
  sportData data; 
  data.applicationId = 0x5900;            //Set the sensor id for the current data poll. Set to 0 to discard the data, skip to the next sensor
  data.value =  temperatureF;                       //Set the sensor value 
  return data;
}

sportData getSensorData1(CustomSPortSensor* intake_pressure) {
  sportData data; 
  data.applicationId = 0x5901;            //Set the sensor id for the current data poll. Set to 0 to discard the data, skip to the next sensor
  data.value = inPressure ;                      //Set the sensor value 
  return data;
}

sportData getSensorData2(CustomSPortSensor* compensation_ratio) {
  sportData data; 
  data.applicationId = 0x5902;            //Set the sensor id for the current data poll. Set to 0 to discard the data, skip to the next sensor
  data.value = ((1000 *  temperatureCorrection * pressureCorrection));                      //Set the sensor value 
  return data;
}

sportData getSensorData3(CustomSPortSensor* solenoid_command) {
  sportData data; 
  data.applicationId = 0x5903;            //Set the sensor id for the current data poll. Set to 0 to discard the data, skip to the next sensor
  data.value =  solenoid_pulse;                      //Set the sensor value 
  return data;
}
sportData getSensorData4(CustomSPortSensor* temperature_corr) {
  sportData data; 
  data.applicationId = 0x5904;            //Set the sensor id for the current data poll. Set to 0 to discard the data, skip to the next sensor
  data.value =  (temperatureCorrection *100);                      //Set the sensor value 
  return data;
}

sportData getSensorData5(CustomSPortSensor* pressure_corr) {
  sportData data; 
  data.applicationId = 0x5905;            //Set the sensor id for the current data poll. Set to 0 to discard the data, skip to the next sensor
  data.value =   (pressureCorrection * 100);                      //Set the sensor value 
  return data;
}