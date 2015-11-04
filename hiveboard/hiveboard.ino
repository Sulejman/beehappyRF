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


const int max_payload_size = 32;
char receive_payload[max_payload_size+1];

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
// ************************** **************************************

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
  for (int i = 0; i < 10; i++) {
    loadCellValue += analogRead(analogVoltageFromMuxLoadCellsPin);
  }

  // turn off the voltage on load cell and then do the averaging calculation
  turnLoadCellOff(loadCellNumber);
  loadCellValue /= 10; 

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

String floatToString(float input){
  String result = "";
  int before_ = (int) input;
  Serial.println(before_);
  int before_decimal = (int) ((input - ((float) input)) * 100);
  String after_ = String(before_);
  String after_decimal = String(after_decimal);
  String dot = ".";
  result = after_ + dot + after_decimal;
  return result;
}

void answer(){
  // if there is data ready
    if ( radio.available() )
    {
      // Dump the payloads until we've gotten everything
      uint8_t len;
      bool done = false;
      while (!done)
      {
        // Fetch the payload, and see if this was the last one.
        len = radio.getDynamicPayloadSize();
        done = radio.read( receive_payload, len );

        // Put a zero at the end for easy printing
        receive_payload[len] = 0;

        // Spew it
        printf("Got payload size=%i value=%s\n\r",len,receive_payload);
      }
      
      if(strcmp(receive_payload, "loadCell1") == 0){
          //Serial.println(readLoadCellAnalog(1));
          String payload_s = String(readLoadCellAnalog(1));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"loadCell2") == 0){
          String payload_s = String(readLoadCellAnalog(2));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"loadCell3") == 0){
          String payload_s = String(readLoadCellAnalog(3));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"loadCell4") == 0){
          String payload_s = String(readLoadCellAnalog(4));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"loadCell5") == 0){
          String payload_s = String(readLoadCellAnalog(5));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"loadCell6") == 0){
          String payload_s = String(readLoadCellAnalog(6));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"loadCell7") == 0){
          String payload_s = String(readLoadCellAnalog(7));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"loadCell8") == 0){
          String payload_s = String(readLoadCellAnalog(8));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"loadCell9") == 0){
          String payload_s = String(readLoadCellAnalog(9));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"loadCell10") == 0){
          String payload_s = String(readLoadCellAnalog(10));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"loadCell10") == 0){
          String payload_s = String(readLoadCellAnalog(10));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"loadCell11") == 0){
          String payload_s = String(readLoadCellAnalog(11));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"loadCell12") == 0){
          String payload_s = String(readLoadCellAnalog(12));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"loadCell13") == 0){
          String payload_s = String(readLoadCellAnalog(13));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"loadCell14") == 0){
          String payload_s = String(readLoadCellAnalog(14));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"loadCell15") == 0){
          String payload_s = String(readLoadCellAnalog(15));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"loadCell16") == 0){
          String payload_s = String(readLoadCellAnalog(16));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"readMuxAnalog1") == 0){
          String payload_s = String(readMuxAnalog(1));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"readMuxAnalog2") == 0){
          String payload_s = String(readMuxAnalog(2));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"readMuxAnalog3") == 0){
          String payload_s = String(readMuxAnalog(3));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"readMuxAnalog4") == 0){
          String payload_s = String(readMuxAnalog(4));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"readMuxAnalog5") == 0){
          String payload_s = String(readMuxAnalog(5));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"readMuxAnalog6") == 0 ){
          String payload_s = String(readMuxAnalog(6));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"readMuxAnalog7") == 0){
          String payload_s = String(readMuxAnalog(7));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"readMuxAnalog8") == 0){
          String payload_s = String(readMuxAnalog(8));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"readMuxAnalog9") == 0){
          String payload_s = String(readMuxAnalog(9));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"readMuxAnalog10") == 0){
          String payload_s = String(readMuxAnalog(10));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"readMuxAnalog11") == 0){
          String payload_s = String(readMuxAnalog(11));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"readMuxAnalog12") == 0){
          String payload_s = String(readMuxAnalog(12));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"readMuxAnalog13") == 0){
          String payload_s = String(readMuxAnalog(13));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"readMuxAnalog14") == 0){
          String payload_s = String(readMuxAnalog(14));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"readMuxAnalog15") == 0){
          String payload_s = String(readMuxAnalog(15));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"readMuxAnalog16") == 0){
          String payload_s = String(readMuxAnalog(16));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"feederLevel") == 0){
          String payload_s = String(getFeederLevel());
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"dht22UpTemperature") == 0){
          delay(500);
          String payload_s = String(readTemperatureSensor(1)) ;
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"dht22DownTemperature") == 0){
          delay(500);
          String payload_s = String(readTemperatureSensor(2));
          delay(500);
          Serial.println(readTemperatureSensor(2));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"dht22UpHumidity") == 0){
          String payload_s = String(readHumiditySensor(1));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"dht22DownHumidity") == 0){
          String payload_s = String(readHumiditySensor(2));
          //Serial.println(payload_s);
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"turnVCC5On") == 0){
          turnVCC5On();
          String payload_s = "Turned VCC 5 ON!";
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"turnVCC5Off") == 0){
          turnVCC5Off();
          String payload_s = "Turned VCC 5 OFF!";
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"turnVCC6On") == 0){
          turnVCC6On();
          String payload_s = "Turned VCC 6 ON!";
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"turnVCC6Off") == 0){
          turnVCC6Off();
          String payload_s = "Turned VCC 6 OFF!";
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"turnVCC7On") == 0){
          turnVCC7On();
          String payload_s = "Turned VCC 7 ON!";
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"turnVCC7Off") == 0){
          turnVCC7Off();
          String payload_s = "Turned VCC 7 OFF!";
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"arduinoAnalog2") == 0){
          String payload_s = String(analogRead(2));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"arduinoAnalog3") == 0){
          String payload_s = String(analogRead(3));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"readArduinoDigital0") == 0){
          String payload_s = String(digitalRead(0));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"readArduinoDigital1") == 0){
          String payload_s = String(digitalRead(1));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"readArduinoDigital5") == 0){
          String payload_s = String(digitalRead(5));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"readArduinoDigital6") == 0){
          String payload_s = String(digitalRead(6));
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"writeArduinoDigital0HIGH") == 0){
          digitalWrite(0,HIGH);
          String payload_s = "PIN0=HIGH";
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"writeArduinoDigital1HIGH") == 0){
          digitalWrite(1,HIGH);
          String payload_s = "PIN1=HIGH";
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"writeArduinoDigital5HIGH") == 0){
          digitalWrite(5,HIGH);
          String payload_s = "PIN5=HIGH";
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"writeArduinoDigital6HIGH") == 0){
          digitalWrite(6,HIGH);
          String payload_s = "PIN6=HIGH";
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"writeArduinoDigital0LOW") == 0){
        digitalWrite(0,LOW);
          String payload_s = "PIN0=LOW";
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"writeArduinoDigital1LOW") == 0){
        digitalWrite(1,LOW);
          String payload_s = "PIN1=LOW";
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"writeArduinoDigital5LOW") == 0 ){
        digitalWrite(5,LOW);
          String payload_s = "PIN5=LOW";
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else if(strcmp(receive_payload,"writeArduinoDigital6LOW") == 0){
          digitalWrite(6,LOW);
          String payload_s = "PIN6=LOW";
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      else{
          String payload_s = "Unknown request code!";
          char payload[sizeof(payload_s)];
          payload_s.toCharArray(payload, sizeof(payload_s));
          memcpy(receive_payload, payload, sizeof(payload) );
      }
      
      printf("New payload value=%s\n\r",receive_payload);

      // First, stop listening so we can talk
      radio.stopListening();

      // Send the final one back.
      radio.write( receive_payload, sizeof(receive_payload) );
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
  

  turn33On();
  delay(20);
  radio.begin();
  radio.enableDynamicPayloads();
  radio.setRetries(15,15);
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1,pipes[0]);
  radio.startListening();
  radio.printDetails();
  
}


void loop() {
 delay(100);
answer();
 }
