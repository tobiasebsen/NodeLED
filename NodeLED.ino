/*
 * NodeLED
 * by Tobias Ebsen
 * 
 * An LED controller and Art-Net node for the Teensy platform.
 * 
 */


#include "FastLED.h"
#include <SPI.h>
#include <Ethernet.h>
#include <ArtNode.h>
#include <EEPROM.h>
#include "TeensyMAC.h"

// FastLED types for each output
#define LED_TYPE_OUT1   WS2812
#define LED_TYPE_OUT2   WS2812
#define LED_TYPE_OUT3   WS2812
#define LED_TYPE_OUT4   WS2812

// Set number of color channels per LED (RGB=3, RGBW=4)
#define NUM_CHANNELS_PER_LED1 3
#define NUM_CHANNELS_PER_LED2 3
#define NUM_CHANNELS_PER_LED3 3
#define NUM_CHANNELS_PER_LED4 3

// Set number of LEDs copied from Dmx packet (max. 170 for RGB and 128 for RGBW)
#define NUM_LEDS_PORT1      170
#define NUM_LEDS_PORT2      170
#define NUM_LEDS_PORT3      170
#define NUM_LEDS_PORT4      170

// Set which Art-Net port goes to which LED output
// This is usefull if your LED strips use more than 512 channels
#define OUTPUT_INDEX_PORT1  0
#define OUTPUT_INDEX_PORT2  1
#define OUTPUT_INDEX_PORT3  2
#define OUTPUT_INDEX_PORT4  3

// Set the channel offset (in bytes) of the Art-Net port dmx-data
// This is usefull if your LED strips use more than 512 channels
#define OUTPUT_OFFSET_PORT1  0
#define OUTPUT_OFFSET_PORT2  0
#define OUTPUT_OFFSET_PORT3  0
#define OUTPUT_OFFSET_PORT4  0

// Set max number of LEDs per output (for memory allocation)
// Make sure this is large enough
#define NUM_LEDS_OUT1 200
#define NUM_LEDS_OUT2 200
#define NUM_LEDS_OUT3 200
#define NUM_LEDS_OUT4 200

// Set number of LED outputs. Maximum 4 !
#define NUM_OUTPUTS   4

// Art-Net
#define NUM_PORTS   4
#define NODE_NET    0
#define NODE_SUBNET 0
#define NODE_IP     2 // Class-A network. Art-Net specification recommends using 2 or 10
#define VERSION_HI  0
#define VERSION_LO  4
#define OEM_CODE    0

// Pins
#define PIN_RESET   9
#define PIN_LOC     23
#define PIN_ACT     22

// LED pins
#define PIN_OUT1    5
#define PIN_OUT2    21
#define PIN_OUT3    20
#define PIN_OUT4    6

//////////////////////////////////////////////
// USER CONFIGURATION ENDS HERE
//////////////////////////////////////////////

const uint16_t numChannelsOut[] = {
  (NUM_CHANNELS_PER_LED1*NUM_LEDS_OUT1),
  (NUM_CHANNELS_PER_LED2*NUM_LEDS_OUT2),
  (NUM_CHANNELS_PER_LED3*NUM_LEDS_OUT3),
  (NUM_CHANNELS_PER_LED4*NUM_LEDS_OUT4)
};

const uint16_t numRgbLeds1 = (uint16_t)ceil(numChannelsOut[0]/3.f);
const uint16_t numRgbLeds2 = (uint16_t)ceil(numChannelsOut[1]/3.f);
const uint16_t numRgbLeds3 = (uint16_t)ceil(numChannelsOut[2]/3.f);
const uint16_t numRgbLeds4 = (uint16_t)ceil(numChannelsOut[3]/3.f);
const uint16_t numRgbLeds[] = {
  numRgbLeds1, numRgbLeds2, numRgbLeds3, numRgbLeds4
};

const uint8_t portsOutput[] = {
  OUTPUT_INDEX_PORT1, OUTPUT_INDEX_PORT2, OUTPUT_INDEX_PORT3, OUTPUT_INDEX_PORT4
};

CRGB ledData1[numRgbLeds1];
CRGB ledData2[numRgbLeds2];
CRGB ledData3[numRgbLeds3];
CRGB ledData4[numRgbLeds4];
const CRGB * ledData[] = {
  ledData1, ledData2, ledData3, ledData4
};

byte * dataPort1 = (byte*)ledData[OUTPUT_INDEX_PORT1] + OUTPUT_OFFSET_PORT1;
byte * dataPort2 = (byte*)ledData[OUTPUT_INDEX_PORT2] + OUTPUT_OFFSET_PORT2;
byte * dataPort3 = (byte*)ledData[OUTPUT_INDEX_PORT3] + OUTPUT_OFFSET_PORT3;
byte * dataPort4 = (byte*)ledData[OUTPUT_INDEX_PORT4] + OUTPUT_OFFSET_PORT4;
byte * dataPort[] = {
  dataPort1, dataPort2, dataPort3, dataPort4
};

uint16_t numChannelsPerLedOut[] = {
  NUM_CHANNELS_PER_LED1, NUM_CHANNELS_PER_LED2, NUM_CHANNELS_PER_LED3, NUM_CHANNELS_PER_LED4
};

// Set max number of channels of DMX data to use for each port (default 512)
const uint16_t numChannelsPort1 = NUM_LEDS_PORT1 * numChannelsPerLedOut[OUTPUT_INDEX_PORT1];
const uint16_t numChannelsPort2 = NUM_LEDS_PORT2 * numChannelsPerLedOut[OUTPUT_INDEX_PORT2];
const uint16_t numChannelsPort3 = NUM_LEDS_PORT3 * numChannelsPerLedOut[OUTPUT_INDEX_PORT3];
const uint16_t numChannelsPort4 = NUM_LEDS_PORT4 * numChannelsPerLedOut[OUTPUT_INDEX_PORT4];
uint16_t numChannelsPort[] = {
  numChannelsPort1, numChannelsPort2, numChannelsPort3, numChannelsPort4
};


byte udpBuffer[600];
EthernetUDP udp;
ArtConfig config;
ArtNode node(config, sizeof(udpBuffer), udpBuffer);

bool locate = false;
unsigned long locateTimer = 0;
bool artsync = false;
unsigned long artsyncTimer = 0;

//////////////////////////////////////////////
// SETUP()
//////////////////////////////////////////////
void setup() {

  // Try to load configuration from EEPROM
  bool initConfig = !loadConfig(&config, sizeof(ArtConfig));

  // Initialize default configuration if no configuration was found
  if (initConfig) {
    memset(&config, 0, sizeof(ArtConfig));
    mac_addr mac;
    for (int i=0; i<6; i++) {
      config.mac[i] = mac.m[i];
    }
    config.ip[0] = NODE_IP;
    config.ip[1] = config.mac[3] + (OEM_CODE & 0xFF);
    config.ip[2] = config.mac[4];
    config.ip[3] = config.mac[5];
    config.mask[0] = 255;
    config.udpPort = 0x1936;
    config.net = NODE_NET;
    config.subnet = NODE_SUBNET;
    strcpy(config.shortName, "NodeLED");
    strcpy(config.longName, "NodeLED");
    config.numPorts = NUM_PORTS;
    for (int i=0; i<config.numPorts; i++) {
      config.portTypes[i] = PortTypeDmx | PortTypeOutput;
      config.portAddrOut[i] = i;
    }
    config.verHi = VERSION_HI;
    config.verLo = VERSION_LO;

    saveConfig(&config, sizeof(ArtConfig));
  }

#ifdef PIN_RESET
  pinMode(PIN_RESET, OUTPUT);
  digitalWrite(PIN_RESET, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_RESET, HIGH);
  delay(150);
#endif

  // Setup pins
  pinMode(PIN_LOC, OUTPUT);
  pinMode(PIN_ACT, OUTPUT);
  digitalWrite(PIN_LOC, HIGH);
  digitalWrite(PIN_ACT, LOW);


  // Open Ethernet connection
  IPAddress gateway(config.ip[0], 0, 0, 1);
  IPAddress subnet(config.mask);
  Ethernet.begin(config.mac, config.ip,  gateway, gateway, subnet);
  udp.begin(config.udpPort);


  // Create 4 outputs with different lengths (can be tweaked)
  FastLED.addLeds<LED_TYPE_OUT1, PIN_OUT1, RGB>(ledData1, numRgbLeds1);
  FastLED.addLeds<LED_TYPE_OUT2, PIN_OUT2, RGB>(ledData2, numRgbLeds2);
  FastLED.addLeds<LED_TYPE_OUT3, PIN_OUT3, RGB>(ledData3, numRgbLeds3);
  FastLED.addLeds<LED_TYPE_OUT4, PIN_OUT4, RGB>(ledData4, numRgbLeds3);
  FastLED.setDither(0);

  // Set all LEDs to zero (black)
  for (int i=0; i<NUM_OUTPUTS; i++) {
    memset((void*)ledData[i], 0, numRgbLeds[i] * 3);
    FastLED[i].show(ledData[i], numRgbLeds[i], 255);
  }

  digitalWrite(PIN_LOC, LOW);
}

//////////////////////////////////////////////
// LOOP()
//////////////////////////////////////////////
void loop() {

  if (udp.parsePacket()) {
    digitalWrite(PIN_ACT, HIGH);

    // First read the header to make sure it's Art-Net
    unsigned int n = udp.read(udpBuffer, sizeof(ArtHeader));
    if (n >= sizeof(ArtHeader)) {
      
      // Check packet ID
      if (node.isPacketValid()) {
        
        // Read the rest of the packet
        udp.read(udpBuffer + sizeof(ArtHeader), udp.available());

        // Package Op-Code determines type of packet
        switch (node.getOpCode()) {

          // Poll packet. Send poll reply.
          case OpPoll:
            node.createPollReply();
            udpSend(udpBuffer, sizeof(ArtPollReply));
            break;

          // DMX packet
          case OpDmx: {
              ArtDmx* dmx = (ArtDmx*)udpBuffer;
              int len = dmx->getLength();
              int port = node.getPort(dmx->Net, dmx->SubUni);
              
              if (port >= 0 && port < NUM_PORTS) {

                int output = portsOutput[port];
                if (output >= 0 && output < 4) {

                  // Copy dmx data
                  memcpy(dataPort[port], dmx->Data, min(len, numChannelsPort[port]));
                  
                  if (!artsync) {
                    FastLED[output].show(ledData[output], numRgbLeds[output], 255);
                  }
                }
              }
          }
          break;

          // SYNC packet
          case OpSync: {
            for (int i=0; i<NUM_OUTPUTS; i++) {
              FastLED[i].show(ledData[i], numRgbLeds[i], 255);
            }
            artsync = true;
            artsyncTimer = millis();
          }
          break;

          case OpAddress: {
            ArtAddress * address = (ArtAddress*)udpBuffer;

            node.handleAddress(address);

            if (address->Command == 0x04) {
              locate = true;
              locateTimer = millis();
              digitalWrite(PIN_LOC, HIGH);
            } else {
              locate = false;
              digitalWrite(PIN_LOC, LOW);
            }

            saveConfig(&config, sizeof(ArtConfig));
            loadConfig(&config, sizeof(ArtConfig));
            node.createPollReply();
            udpSend(udpBuffer, sizeof(ArtPollReply));
          }
          break;
          
        }
      }
    }

    digitalWrite(PIN_ACT, LOW);
  }

  // Check if sync has timed out
  if (artsync && millis() - artsyncTimer >= 4000)
    artsync = false;

  // Handle locate blinking
  if (locate) {
    long t = millis() - locateTimer;
    if (t < 100)
      digitalWrite(PIN_LOC, HIGH);
    if (t > 100)
      digitalWrite(PIN_LOC, LOW);
    if (t > 200)
      locateTimer = millis();
  }
}

//////////////////////////////////////////////
// loadConfig()
//////////////////////////////////////////////
bool loadConfig(void *cfg, int nbytes) {
  if (EEPROM.read(0) == 'c' &&
      EEPROM.read(1) == 'f' &&
      EEPROM.read(2) == 'g') {

    for (int i=0; i<nbytes; i++) {
      *((char*)cfg + i) = EEPROM.read(3 + i);
    }
    return true;
  }
  return false;
}

//////////////////////////////////////////////
// saveConfig()
//////////////////////////////////////////////
void saveConfig(void *cfg, int nbytes) {
  EEPROM.write(0, 'c');
  EEPROM.write(1, 'f');
  EEPROM.write(2, 'g');
  for (int i=0; i<nbytes; i++) {
    EEPROM.write(3 + i, *((char*)cfg + i));
  }
}

//////////////////////////////////////////////
void udpSend(byte* buffer, int length) {
  udp.beginPacket(node.broadcastIP(), config.udpPort);
  udp.write(buffer, length);
  udp.endPacket();
}

