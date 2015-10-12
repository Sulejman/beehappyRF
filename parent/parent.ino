#include <ShiftRegister74HC595.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9,10);

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

int carrier[16] = {0,0,0,0,0,0,0,0,1,3,1,0,0,0,0,0};
int response[16];

// create shift register object (number of shift registers, data pin, clock pin, latch pin)
int numOfShiftReg = 3;
int sRDataPin = 2;
int sRClockPin = 3;
int sRLatchPin = 4;
ShiftRegister74HC595 sR (numOfShiftReg, sRDataPin, sRClockPin, sRLatchPin); 


void turn33On();
void turn33Off();
void ask();
void printArray(int array[16]);


//FUNCTIONS

// function to turn on the 3.3V for the NRF24L01+
void turn33On() {
  sR.set(7, LOW);
}

// function to turn off the 3.3V for the NRF24L01+
void turn33Off() {
  sR.set(7, HIGH);
}

void ask(){
  
    for(int i=0; i<16; i++){
          response[i] = 0;
    }
  
  // First, stop listening so we can talk.
    radio.stopListening();

    // Take the time, and send it.  This will block until complete
    Serial.print("Now sending carrier...");
    bool ok = radio.write( &carrier, sizeof(carrier) );
    
    if (ok)
      Serial.print("ok...");
    else
      Serial.print("failed.\n\r");

    // Now, continue listening
    radio.startListening();

    // Wait here until we get a response, or timeout (250ms)
    unsigned long started_waiting_at = millis();
    bool timeout = false;
    while ( ! radio.available() && ! timeout )
      if (millis() - started_waiting_at > 120000 )
        timeout = true;

    // Describe the results
    if ( timeout )
    {
      Serial.print("Failed, response timed out.\n\r");
    }
    else
    {
      // Grab the response, compare, and send to debugging spew
      radio.read( &response, sizeof(response) );

      // Spew it
      Serial.print("Got response:\t");
      printArray(response);
    }

    // Try again 1s later
    delay(1000);

}

void printArray(int array[16]){
    for(int i = 0; i < 16; i++){
        Serial.print(array[i]);
        Serial.print("\t");
    }
    Serial.print("\n");
}

//END OF FUNCTIONS

void setup(void)
{
  Serial.begin(57600);
  //printf_begin();
  turn33On();

  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);

  // optionally, reduce the payload size.  seems to
  // improve reliability  
  //radio.setPayloadSize(8);
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);
  radio.startListening();
  Serial.println("evo");
  radio.printDetails();
  Serial.println("ovdje");
}

void loop(void)
{
   Serial.println("Getting weights from 1-8");
   carrier[9] = 1;
   carrier[10] = 0;
   ask();
   Serial.println("Getting weights from 9-16");
   carrier[9] = 2;
   carrier[10] = 0;
   ask();
   Serial.println("Getting DHT values");
   carrier[9] = 3;
   carrier[10] = 0;
   ask();
   Serial.println("Getting feeder value.");
   carrier[9] = 4;
   carrier[10] = 0;
   ask();
   Serial.println("Calibrating 1-8");
   carrier[9] = 7;
   carrier[10] = 0;
   ask();
   Serial.println("Calibrating 9-16");
   carrier[9] = 8;
   carrier[10] = 0;
   ask();
   for(int i = 1; i <17; i++){
    Serial.print("Reading load cell number ");
    Serial.print(i);
    Serial.println("");
    carrier[9] = 13;
    carrier[10] = i;
    ask(); 
   }
}
