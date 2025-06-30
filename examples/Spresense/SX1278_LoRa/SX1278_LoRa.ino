#include <RadioLib.h>
#include "SpresenseHal.h"

// Spresense pins for SX1278
#define LORA_CS    10  // Chip Select pin
#define LORA_IRQ   2   // Interrupt pin
#define LORA_RST   9   // Reset pin
#define LORA_GPIO  3   // GPIO pin (optional)

// create radio instance using the Spresense HAL
SpresenseHal hal;
SX1278 radio = new Module(&hal, LORA_CS, LORA_IRQ, LORA_RST, LORA_GPIO);

// or using LoRa class
// LoRa radio = new LoRa(&hal, LORA_CS, LORA_IRQ, LORA_RST, LORA_GPIO);

void setup() {
  Serial.begin(115200);
  while(!Serial);

  // initialize SX1278 with default settings
  Serial.print(F("[SX1278] Initializing ... "));
  
  // carrier frequency:           915.0 MHz
  // bandwidth:                   125.0 kHz
  // spreading factor:            7
  // coding rate:                 5
  // sync word:                   0x12
  // output power:                17 dBm
  // current limit:               100 mA
  // preamble length:             8 symbols
  // amplifier gain:              0 (automatic gain control)
  int state = radio.begin();
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // set the function that will be called
  // when packet is received
  radio.setPacketReceivedAction(onPacketReceived);
}

void loop() {
  // send a packet
  Serial.print(F("[SX1278] Sending packet ... "));
  
  // you can transmit C-string or Arduino string up to
  // 256 characters long
  int state = radio.transmit("Hello World!");
  
  if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F("success!"));

    // wait for a second before transmitting again
    delay(1000);
  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("too long!"));
  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
}

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void onPacketReceived() {
  // you can read received data as an Arduino String
  String str;
  int state = radio.readData(str);
  
  if (state == RADIOLIB_ERR_NONE) {
    // packet was successfully received
    Serial.println(F("[SX1278] Received packet!"));
    
    // print the data of the packet
    Serial.print(F("[SX1278] Data:\t\t"));
    Serial.println(str);
    
    // print the RSSI (Received Signal Strength Indicator)
    // of the last received packet
    Serial.print(F("[SX1278] RSSI:\t\t"));
    Serial.print(radio.getRSSI());
    Serial.println(F(" dBm"));
    
    // print the SNR (Signal-to-Noise Ratio)
    // of the last received packet
    Serial.print(F("[SX1278] SNR:\t\t"));
    Serial.print(radio.getSNR());
    Serial.println(F(" dB"));
  }
} 