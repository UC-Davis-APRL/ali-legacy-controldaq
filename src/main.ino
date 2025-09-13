#include "ADS1256.h"
#include "StateMachine.h"
#include <inttypes.h>
#include <time.h>
#include <Wire.h>
#include <QNEthernet.h>

/*======== ADC ========*/

const int CS1 = 10;
const int DRDY = 17;
const int PDWN = 14;

ADS1256 adc(DRDY, 2000000, PDWN, CS1, 2.5); //DRDY, SPI speed, SYNC(PDWN), CS, VREF(float).
StateMachine machina(33,32,29,28,31,30,34,4,2,3); // isok, isol, maink, mainl, ventk, ventl, purge, emerg, breakwire, igniter 

long rawConversion = 0; //24-bit raw value
float voltageValue = 0; //human-readable floating point value

int singleEndedChannels[8] = {SING_0, SING_1, SING_2, SING_3, SING_4, SING_5, SING_6, SING_7}; //Array to store the single-ended channels
int inputChannel = 0; //Number used to pick the channel from the above two arrays
char inputMode = ' '; //can be 's' and 'd': single-ended and differential

int pgaValues[7] = {PGA_1, PGA_2, PGA_4, PGA_8, PGA_16, PGA_32, PGA_64}; //Array to store the PGA settings
int pgaSelection = 0; //Number used to pick the PGA value from the above array

unsigned int fireTime = 5000;
unsigned int purgeTime = 5000;
bool firstLoop = 1;

int drateValues[16] =
{
  DRATE_30000SPS,
  DRATE_15000SPS,
  DRATE_7500SPS,
  DRATE_3750SPS,
  DRATE_2000SPS,
  DRATE_1000SPS,
  DRATE_500SPS,
  DRATE_100SPS,
  DRATE_60SPS,
  DRATE_50SPS,
  DRATE_30SPS,
  DRATE_25SPS,
  DRATE_15SPS,
  DRATE_10SPS,
  DRATE_5SPS,
  DRATE_2SPS
}; //Array to store the sampling rates

int drateSelection = 0; //Number used to pick the sampling rate from the above array

String registers[11] =
{
  "STATUS",
  "MUX",
  "ADCON",
  "DRATE",
  "IO",
  "OFC0",
  "OFC1",
  "OFC2",
  "FSC0",
  "FSC1",
  "FSC2"
};//Array to store the registers

int registerToRead = 0; //Register number to be read
int registerToWrite = 0; //Register number to be written
int registerValueToWrite = 0; //Value to be written in the selected register


//==================Ethernet==================//
const int sensor_number = 8;

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
IPAddress ip(192,168,88,250);     // MCU IP
IPAddress subnet(255,255,255,0); // set subnet mask
// IPAddress remote(10,0,0,51);    // PC IP
IPAddress remote(192,168,88,251);
unsigned int localPort = 1682;     // local port to listen on
unsigned int remotePort = 1682;

// An EthernetUDP instance to let us send and receive packets over UDP
using namespace qindesign::network;
EthernetUDP Udp;

int packetSize = 0;

// buffers for receiving and sending data
//each reading is 4 byte
//individual reading timestamp 4 byte
//id 1 byte
//timestamp 4 byte
const int dataPacketSize = sensor_number*4+4+4; 
uint8_t outgoingDataPacketBuffers[dataPacketSize]; // buffer for going out to PC
uint8_t valveStatesBuffer[48];

uint8_t loopCounter = 0;
uint32_t id = 0;
bool valveStateChange = 1;

unsigned long timeElapsed = 0;

void setup() 
{
  Serial.begin(115200); //The value does not matter if you use an MCU with native USB
  delay(1000);

  Serial.println("Serial available");

  machina.initializeMachina();

  // Check for Ethernet hardware present
  if (!Ethernet.begin()) {
    printf("Failed to start Ethernet\r\n");
    return;
  }

  // Listen for link changes
  Ethernet.onLinkState([](bool state) {
    printf("[Ethernet] Link %s\r\n", state ? "ON" : "OFF");
  });

  // start UDP
  Udp.beginWithReuse(localPort);
  delay(1000);


  // initialize adc and set gain + data rate
  adc.InitializeADC();
  delay(1000);

  adc.setPGA(PGA_1);
  delay(1000);
  adc.setDRATE(DRATE_1000SPS);
  delay(1000);

  Serial.println(adc.readRegister(DRATE_REG));
  Serial.println(adc.readRegister(DRATE_REG));
  Serial.println(adc.readRegister(DRATE_REG));
  
  // Serial.print("PGA: ");
  // Serial.println(adc.readRegister(IO_REG));
  // delay(500);

  // Serial.print("DRATE: ");
  // Serial.println(adc.readRegister(DRATE_REG));
  // delay(500);

  // initialize outgoingDataPacketBuffers
  for(int i = 0; i<dataPacketSize; i++)
  {
    outgoingDataPacketBuffers[i] = 0;
  }

  //Freeze the display for 1 sec
  delay(1000);
}


// the function converts a 32 bits int array into 8 bit int for udp compatibility
// void bufferConversion32(uint32_t* arr, int size, uint8_t* buffer)
// {
//   uint8_t startByte; // the position to begin shifting; multiple of 4 because there are
//                      // 4 bytes in a 32 bits int

//   // for loop to index next value in 32 bit array
//   for (int i = 0; i<size; i++)
//   {
//     startByte = 4*i;
//     // for loop to shifts 4 bytes individually from int32 
//     for(int j = 0; j<4; j++)
//     {
//       buffer[startByte + j] = (arr[i] >> (8*(3-j))) & 0xFF;
//     }
//   }
// }

void loop() 
{
  uint32_t commandBuffer[2] = {0,0};
  uint32_t machineState = (uint32_t) machina.getState();
  uint32_t valveStates[11] = {2,machina.isok_state,machina.isol_state,machina.maink_state,
                            machina.mainl_state,machina.ventk_state,machina.ventl_state,machina.purge_state,
                            machina.getBreakWire(),machina.getKeySwitch(),machineState};

  if(machina.valveStateChange || firstLoop)
  {

    for (int i = 0; i<11; i++)
    {   
      uint8_t startByte = 4*i;

      // for loop to shifts 4 bytes individually from int32 
      for(int j = 0; j<4; j++)
      {
        valveStatesBuffer[startByte + j] = (valveStates[i] >> (8*(3-j))) & 0xFF;
      }
    }


    for (int i = 0; i<4; i++)
    {
      valveStatesBuffer[(48-4)+i] = (millis() >> (8*(3-i))) & 0xFF;
    }

    Udp.send(remote,remotePort,valveStatesBuffer,48);
    machina.valveStateChange = 0;
    firstLoop = 0;
  }

  packetSize = Udp.parsePacket(); // check to see if we receive any command

  if(packetSize > 0)
  {
    // pointer address to received data array from UDP
    const uint8_t* packetBuffer = Udp.data(); 

    commandBuffer[0] = packetBuffer[3]; 
    commandBuffer[1] = packetBuffer[7];
  }

  for (int i = 0; i<4; i++)
  {
    outgoingDataPacketBuffers[(dataPacketSize-4)+i] = (millis() >> (8*(3-i))) & 0xFF;
  }

  // filling individual readings into an array
  for (int i = 0; i < sensor_number; i++)
  {
    long tempData;
    uint8_t startByte = (i*4) + 4;
    tempData = adc.cycleSingle();
    // tempData = random(100);
    
    for (int j = 0; j<4; j++)
    {
      outgoingDataPacketBuffers[startByte+j] = (tempData >> (8*(3-j))) & 0xFF;
    }
  }

  for (int j = 0; j<4; j++)
  {
    outgoingDataPacketBuffers[j] = (id >> (8*(3-j))) & 0xFF;
  }

  // send sensor data packet
  Udp.send(remote,remotePort,outgoingDataPacketBuffers,dataPacketSize);

  // activate state machine
  timeElapsed = millis() - machina.referenceTime;
  machina.processCommand(commandBuffer[1], fireTime, purgeTime, timeElapsed, 0);
}
