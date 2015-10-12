#include <CapacitiveSensor.h>
#include <ShiftRegister74HC595.h>
#include <DHT.h>
#include <SPI.h>         //needed for nrf connection
#include <nRF24L01.h>
#include <RF24.h>
#include "printf.h"

// create shift register object (number of shift registers, data pin, clock pin, latch pin)
int numOfShiftReg = 3;
int sRDataPin = 2;
int sRClockPin = 3;
int sRLatchPin = 4;
int weightCalibration[16];  
int emptyCells[16];
int weights[16];
float dht[16];
ShiftRegister74HC595 sR (numOfShiftReg, sRDataPin, sRClockPin, sRLatchPin); 

//pins for capacitive sensor
CapacitiveSensor   cs_5_6 = CapacitiveSensor(5,6);
// 10M resistor between pins 5 & 6, pin 6 is sensor pin, add a wire and or foil if desired

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9,10);

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

int response[16];

// defines the type of sensor and initialize it
int dhtSensorOnePin = 8;
int dhtSensorTwoPin = 7;
#define DHTTYPE DHT22
DHT dhtSensorOne(dhtSensorOnePin, DHTTYPE);
DHT dhtSensorTwo(dhtSensorTwoPin, DHTTYPE);

// define the analog voltage pins
int analogVoltageFromMuxLoadCellsPin = A0;
int analogVoltageFromMux = A1;


// definitions for all the upcoming functions
void turnLoadCellOn(int loadCellNumber);
void turnLoadCellOff(int loadCellNumber);
void turn33On();
void turn33Off();
void turnVCC7On();
void turnVCC7Off();
void turnVCC6On();
void turnVCC6Off();
void turnVCC5On();
void turnVCC5Off();

void muxLoadCell(unsigned char loadCellSignalLines);

float readTemperatureSensor(int sensorNumber);
float readHumiditySensor(int sensorNumber);

int readLoadCellAnalog(int loadCellNumber);
int readMuxAnalog(int muxLocation);

// START OF FUNCTIONS
// ****************************************************************

// function to turn on the voltage for the load celll
void turnLoadCellOn(int loadCellNumber) {
  if(loadCellNumber <= 8) {
    sR.set(loadCellNumber+15 , LOW);
  } else {
    sR.set(loadCellNumber-1, LOW);
  }
}

// function to turn off the voltage for the load celll
void turnLoadCellOff(int loadCellNumber) {
  if(loadCellNumber <= 8) {
    sR.set(loadCellNumber+15 , HIGH);
  } else {
    sR.set(loadCellNumber-1, HIGH);
  }
}

// function to turn on the 3.3V for the NRF24L01+
void turn33On() {
  sR.set(7, LOW);
}

// function to turn off the 3.3V for the NRF24L01+
void turn33Off() {
  sR.set(7, HIGH);
}
 
// function to turn VCC on 7 on
void turnVCC7On(){
  sR.set(6, LOW);
}

// function to turn VCC on 7 off
void turnVCC7Off(){
  sR.set(6, HIGH);
}

// function to turn VCC on 6 on
void turnVCC6On(){
  sR.set(5, LOW);
}

// function to turn VCC on 6 off
void turnVCC6Off(){
  sR.set(5, HIGH);
}

// function to turn VCC on 5 on
void turnVCC5On(){
  sR.set(4, LOW);
}

// function to turn VCC on 5 off
void turnVCC5Off(){
  sR.set(4, HIGH);
}

// function for addressing the load cell signal lines to the amplifier
// which line do we want to let pass
void muxLoadCell(unsigned char loadCellSignalLines) {
  // subtract -1 from loadCellSignalLines so that you enter a value
  // between 1 and 16 e.q. 1=0 and 16=15
  loadCellSignalLines--;
  // output things on the shift registers so that the
  // multiplexer is addressed correctly for each loadCellSignalLines
  sR.set(0, (loadCellSignalLines&0x01));
  sR.set(1, (loadCellSignalLines&0x02)>>1);
  sR.set(2, (loadCellSignalLines&0x04)>>2);
  sR.set(3, (loadCellSignalLines&0x08)>>4);
}

// function that returns the temperature
// 1 for sensor 1 and 2 for sensor two, otherwise -999
float readTemperatureSensor(int sensorNumber) {
  float temp;
  if(sensorNumber == 1) {
     temp = dhtSensorOne.readTemperature();
  } else if(sensorNumber == 2) {
     temp = dhtSensorTwo.readTemperature();
  } 
  if(isnan(temp)) {
    return -999;
  } else {
    return temp;
  }
}

// function that returns the humidity
// 1 for sensor 1 and 2 for sensor two, otherwise -999
float readHumiditySensor(int sensorNumber) {
  float humidity;
  if(sensorNumber == 1) {
    humidity = dhtSensorOne.readHumidity();
  } else if(sensorNumber == 2) {
    humidity = dhtSensorTwo.readHumidity();
  } 
  if(isnan(humidity)) {
    return -999;
  } else {
    return humidity;
  }  
}

// function to read analog voltage from load cells
// enter 1-16, returns int value of voltage
int readLoadCellAnalog(int loadCellNumber) {
  // turn the voltage on for the load cell
  turnLoadCellOn(loadCellNumber);
  //delay(1000);
  // select the correct load cell
  muxLoadCell(loadCellNumber);

  // read the voltage 10 times and then calculate
  // the average for better approximation
  int loadCellValue = 0; 
  //for (int i = 0; i < 10; i++) {
    loadCellValue += analogRead(analogVoltageFromMuxLoadCellsPin);
  //}

  // turn off the voltage on load cell and then do the averaging calculation
  turnLoadCellOff(loadCellNumber);
  //loadCellValue /= 10; 

  return loadCellValue;
}

// function to read the analog value from the unused mux
int readMuxAnalog(int muxInputPin) {

  // select the correct mux input
  muxLoadCell(muxInputPin);
  
  // read the voltage 10 times and then calculate
  // the average for better approximation
  int muxAnalogValue = 0; 
  for (int i = 0; i < 20; i++) {
    muxAnalogValue += analogRead(analogVoltageFromMux);
  }
  // do the averaging calculation
  muxAnalogValue /= 20; 
  return muxAnalogValue;
}

//function to get feeder level - analog value
int getFeederLevel(){
  int feeder_sum = 0;
  int feeder_level;
  for(int i=0; i<10; i++){
      feeder_sum += cs_5_6.capacitiveSensor(30);
  }
  feeder_level = feeder_sum/10;
  return feeder_level;
}

//function to calibrate load cells
void calibrateLoadCells(){
  
  Serial.println("Calibrating load cells ...");
  
  for(int i=1; i < 17; i++){
    delay(1000);
    readLoadCellAnalog(i);
    Serial.println("...");
  }
  
  for(int i=0; i < 16; i++){
    delay(1000);
    weightCalibration[i] = readLoadCellAnalog(i+1);
    Serial.println("...");
  }
  
}

void getUpWeight(int array[16]){
  for(int i=0; i < 8; i++){
    delay(500);
    weights[i] = readLoadCellAnalog(i+1);
    array[i] = weights[i];
    Serial.println(weights[i]);
  }
}

void getDownWeight(int array[16]){
  for(int i=0; i < 8; i++){
    delay(500);
    weights[i] = readLoadCellAnalog(i+9);
    array[i] = weights[i];
    Serial.println(weights[i]);
  }
}

void getWeightInGrams(int values[]){
  Serial.println("Weight in grams:");
  for(int i=0; i<17; i++){
    weights[i] = values[i] * 10 / 1.12;
    Serial.println(weights[i]);
  }
}

void printArray(int array[16]){
    for(int i = 0; i < 16; i++){
        printf("%i\t", array[i]);
    }
}

void getDhtValuesToSend(int array[16]){
  
  float t1 = readTemperatureSensor(1);
  float t2 = readTemperatureSensor(2);
  float h1 = readHumiditySensor(1);
  float h2 = readHumiditySensor(2);
  
  Serial.println(t1);
  Serial.println(t2);
  Serial.println(h1);
  Serial.println(h2);
  
  array[0] = (int) t1;
  array[1] = (int) ((t1 - ((float) t1)) * 100);
  array[2] = (int) t2;
  array[3] = (int) ((t2 - ((float) t2)) * 100);
  array[4] = (int) h1;
  array[5] = (int) ((h1 - ((float) h1)) * 100);
  array[6] = (int) h2;
  array[7] = (int) ((h2 - ((float) h2)) * 100);
  
}

void getMuxValueToSend(int pin, int array[16]){

  float value = readMuxAnalog(pin);
  array[0] = (int) value;
  array[1] = (int) ((value - ((float) value)) * 100);
  
}


void answer(){
  
    for(int i=0; i<16; i++){
          response[i] = 0;
    }
  
  // if there is data ready
    if ( radio.available() )
    {
      // Dump the payloads until we've gotten everything
      unsigned long got_time;
      bool done = false;
      while (!done)
      {
        // Fetch the payload, and see if this was the last one.
        done = radio.read( &response, sizeof(response) );

        // Spew it
        printf("Got payload:\t");
        printArray(response);
        printf("\n");
        

	// Delay just a little bit to let the other unit
	// make the transition to receiver
	delay(20);
      }

      // First, stop listening so we can talk
      radio.stopListening();
      int request_code = response[9];
      switch(request_code){
        case 1 : getUpWeight(response);
          break;
        case 2 : getDownWeight(response);
          break;
        case 3 : getDhtValuesToSend(response);
          break;
        case 4 : response[0] = getFeederLevel();
          break;
        case 5 : turnVCC5On();
          break;
        case 6 : turnVCC5Off();
        case 7 : calibrateLoadCells(); 
                  for(int i = 0; i < 8; i++){
                  response[i] = weightCalibration[i];
                  } 
                  break;
        case 8 : calibrateLoadCells(); 
                  for(int i = 9; i < 16; i++){
                  response[i-8] = weightCalibration[i];
                  } 
                  break;
        case 9 : turnVCC6On();
          break;
        case 10 : turnVCC6Off();
          break;
        case 11 : turnVCC7On();
          break;
        case 12 : turnVCC7On();
          break;
        case 13: readLoadCellAnalog(response[10]); 
                 response[0] = readLoadCellAnalog(response[10]);
          break;
        case 14: getMuxValueToSend(response[10], response);
          break;
        
        
        default : response[0] = -1;
      }

      // Send the final one back.
      radio.write( &response, sizeof(response) );
      printf("Sent response.\n\r");

      // Now, resume listening so we catch the next packets.
      radio.startListening();
    }

}



// END OF FUNCTIONS
// ****************************************************************


// these settings must be fullfilled, do not change!!!
void setup() {   
  // set all output pins HIGH on the shift register otherwise
  // there might be random outputs
  sR.setAllHigh();
  

  // start the temperature and humidity sensors
  dhtSensorOne.begin();
  dhtSensorTwo.begin();
  
  delay(150);
  Serial.begin(57600);
  printf_begin();
  
  //calibrateLoadCells();

  turn33On();
  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);
  //radio.setChannel(99);

  // optionally, reduce the payload size.  seems to
  // improve reliability
  //radio.setPayloadSize(8);

  //
  // Open pipes to other nodes for communication
  //

    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1,pipes[0]);

  //
  // Start listening
  //
  //radio.setChannel(40);
  radio.startListening();

  //
  // Dump the configuration of the rf unit for debugging
  //

  radio.printDetails();
  
}


void loop() {

 
 answer();
 
 }
