#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <RadioLib.h>
#include "SpresenseNuttxHal.h"

// Pin definitions for SX1278
#define PIN_CS    10  // Chip Select
#define PIN_DIO0  11  // Interrupt
#define PIN_RST   12  // Reset
#define PIN_DIO1  13  // Optional interrupt

// Create radio instance
SpresenseNuttxHal hal;
SX1278 radio = new Module(PIN_CS, PIN_DIO0, PIN_RST, PIN_DIO1, hal);

// Flag to indicate that a packet was received
volatile bool receivedFlag = false;

// Disable interrupt when it's not needed
volatile bool enableInterrupt = true;

// This function is called when a complete packet is received by the module
void onPacketReceived(void) {
  // Check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }

  // We got a packet, set the flag
  receivedFlag = true;
}

int main(int argc, char *argv[]) {
  printf("SX1278 LoRa Test\n");

  // Initialize the radio
  printf("Initializing ... ");
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    printf("success!\n");
  } else {
    printf("failed, code %d\n", state);
    return 1;
  }

  // Set the function that will be called when packet is received
  radio.setPacketReceivedAction(onPacketReceived);

  // Start listening for packets
  printf("Starting to listen ... ");
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    printf("success!\n");
  } else {
    printf("failed, code %d\n", state);
    return 1;
  }

  // Main loop
  while(1) {
    // Check if the flag is set
    if(receivedFlag) {
      // Disable the interrupt service routine while processing the data
      enableInterrupt = false;

      // Reset flag
      receivedFlag = false;

      // Read received data
      uint8_t data[256];
      int dataLength = 0;
      state = radio.readData(data, &dataLength);

      if (state == RADIOLIB_ERR_NONE) {
        // Packet was successfully received
        printf("Received packet! Length: %d bytes\n", dataLength);

        // Print the data
        printf("Data: ");
        for(int i = 0; i < dataLength; i++) {
          printf("%c", data[i]);
        }
        printf("\n");

        // Print RSSI (Received Signal Strength Indicator)
        printf("RSSI: %.2f dBm\n", radio.getRSSI());

        // Print SNR (Signal-to-Noise Ratio)
        printf("SNR: %.2f dB\n", radio.getSNR());
      } else {
        // Some error occurred
        printf("Failed to read data, code %d\n", state);
      }

      // Start listening for the next packet
      radio.startReceive();

      // Enable the interrupt service routine
      enableInterrupt = true;
    }

    // Sleep for a bit
    usleep(10000);  // 10ms
  }

  return 0;
} 