#include <SPI.h>
#include <mcp2515.h>
#include "Utils.h"
#define CAN_ID  0x000000F3
#define ENCODER_PIN 3
#define CAN0_INT  2               // Set INT to pin 2  

// Motor variables
const uint8_t sample_time = 20;
const double kSecondInMillis = 1000.0;
const double kSecondsInMinute = 60.0;
const double kSamplesPerSecond = kSecondInMillis / sample_time;
const double kSamplesPerMinute = kSamplesPerSecond * kSecondsInMinute;
const double kEncoderTicsRev = 900.0;
const double kMotorRadius = 0.032;
int encoder_tics = 0;


// CAN TX Variables
struct can_frame canMsgS;
unsigned long prevTX = 0;                   // Variable to store last execution time
uint8_t data[] = {0x00, 0x00, 0x00, 0x00};;// Generic CAN data to send

union {
  double val;
  uint8_t val_arr[sizeof(double)];
}double_bytes;
              
MCP2515 mcp2515(10);              // Set CS to pin 10

void setup() {
  //Setup the CAN_TX message
  canMsgS.can_id = CAN_ID | CAN_EFF_FLAG;   canMsgS.can_dlc = sizeof(data); 
  canMsgS.data[0] = data[0];  canMsgS.data[1] = data[1];  canMsgS.data[2] = data[2];  canMsgS.data[3] = data[3]; 
  //canMsgS.data[4] = data[4];  canMsgS.data[5] = data[5];  canMsgS.data[6] = data[6];  canMsgS.data[7] = data[7];
  
  Serial.begin(9600);
  
  SPI.begin();
  if( mcp2515.reset() == MCP2515 :: ERROR_OK ){
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    //mcp2515.setLoopbackMode();
    Serial.println("MCP2515 Initialized Successfully!");
  }
  else
    Serial.println("Error Initializing MCP2515...");
  
  pinMode(CAN0_INT, INPUT);                     // Configuring pin for /INT input
  pinMode(ENCODER_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), Utils::handleEncoder, RISING);

  double_bytes.val = 0;
}

 void loop() {
  if(millis() - prevTX >= sample_time){           // Send this at a one second interval.
    prevTX = millis();
    Utils::updateSpeed(double_bytes.val);

    Serial.print(" ");
    Serial.print(double_bytes.val);

    for (uint8_t i = 0; i < sizeof(double_bytes.val_arr); i++)
      canMsgS.data[i] = double_bytes.val_arr[i];

    Serial.println("");
    
    if( mcp2515.sendMessage(&canMsgS) == MCP2515 :: ERROR_OK )
      Serial.print("CAN Message Sent Successfully!");
    else
      Serial.print("Error Sending CAN Message...");
  }
}
