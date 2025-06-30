#include <RadioLib.h>
#include "SpresenseNuttxHal.h"

// Spresense NuttX HAL instance
SpresenseNuttxHal hal;

// SX1278 module instance
SX1278 radio = new Module(hal, 10, 2, 3);

void setup() {
    printf("Spresense NuttX RadioLib Example\n");
    printf("================================\n");
    
    // Initialize the HAL
    hal.init();
    
    // Initialize SX1278
    printf("Initializing SX1278...\n");
    int state = radio.begin();
    if (state == RADIOLIB_ERR_NONE) {
        printf("SX1278 initialized successfully!\n");
    } else {
        printf("SX1278 initialization failed! Error code: %d\n", state);
        return;
    }
    
    // Set frequency to 433 MHz
    state = radio.setFrequency(433.0);
    if (state == RADIOLIB_ERR_NONE) {
        printf("Frequency set to 433 MHz\n");
    } else {
        printf("Failed to set frequency! Error code: %d\n", state);
    }
    
    // Set bandwidth to 125 kHz
    state = radio.setBandwidth(125.0);
    if (state == RADIOLIB_ERR_NONE) {
        printf("Bandwidth set to 125 kHz\n");
    } else {
        printf("Failed to set bandwidth! Error code: %d\n", state);
    }
    
    // Set spreading factor to 7
    state = radio.setSpreadingFactor(7);
    if (state == RADIOLIB_ERR_NONE) {
        printf("Spreading factor set to 7\n");
    } else {
        printf("Failed to set spreading factor! Error code: %d\n", state);
    }
    
    // Set coding rate to 5
    state = radio.setCodingRate(5);
    if (state == RADIOLIB_ERR_NONE) {
        printf("Coding rate set to 5\n");
    } else {
        printf("Failed to set coding rate! Error code: %d\n", state);
    }
    
    // Set output power to 10 dBm
    state = radio.setOutputPower(10.0);
    if (state == RADIOLIB_ERR_NONE) {
        printf("Output power set to 10 dBm\n");
    } else {
        printf("Failed to set output power! Error code: %d\n", state);
    }
    
    printf("Radio configuration complete!\n");
}

void loop() {
    // Transmit a message
    printf("Transmitting packet...\n");
    int state = radio.transmit("Hello from Spresense NuttX!");
    if (state == RADIOLIB_ERR_NONE) {
        printf("Packet transmitted successfully!\n");
    } else {
        printf("Failed to transmit packet! Error code: %d\n", state);
    }
    
    // Wait 5 seconds
    hal.delay(5000);
    
    // Try to receive a packet
    printf("Receiving packet...\n");
    String str;
    state = radio.receive(str);
    if (state == RADIOLIB_ERR_NONE) {
        printf("Received packet: %s\n", str.c_str());
        printf("RSSI: %.2f dBm\n", radio.getRSSI());
        printf("SNR: %.2f dB\n", radio.getSNR());
    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
        printf("No packet received (timeout)\n");
    } else {
        printf("Failed to receive packet! Error code: %d\n", state);
    }
    
    // Wait 5 seconds before next cycle
    hal.delay(5000);
}

int main() {
    setup();
    
    while (1) {
        loop();
    }
    
    return 0;
} 