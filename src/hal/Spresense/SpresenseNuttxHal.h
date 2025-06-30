#ifndef SPRESENSE_NUTTX_HAL_H
#define SPRESENSE_NUTTX_HAL_H

#if defined(RADIOLIB_BUILD_SPRESENSE)

// include RadioLib
#include <RadioLib.h>

// include NuttX headers
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/gpio/gpio.h>
#include <nuttx/irq.h>
#include <nuttx/pwm.h>
#include <nuttx/timers/timer.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <time.h>

// Spresense-specific constants
#define SPRESENSE_GPIO_INPUT          (0)
#define SPRESENSE_GPIO_OUTPUT         (1)
#define SPRESENSE_GPIO_LOW            (0)
#define SPRESENSE_GPIO_HIGH           (1)
#define SPRESENSE_GPIO_RISING         (2)
#define SPRESENSE_GPIO_FALLING        (3)

// Spresense hardware abstraction layer for NuttX
// Must inherit from RadioLibHal and implement all virtual methods
class SpresenseNuttxHal : public RadioLibHal {
  public:
    // Constructor with SPI configuration
    SpresenseNuttxHal(const char* spi_dev = "/dev/spi0", 
                      uint32_t spi_speed = 2000000,
                      uint32_t spi_mode = 0)
      : RadioLibHal(SPRESENSE_GPIO_INPUT, SPRESENSE_GPIO_OUTPUT, 
                    SPRESENSE_GPIO_LOW, SPRESENSE_GPIO_HIGH, 
                    SPRESENSE_GPIO_RISING, SPRESENSE_GPIO_FALLING),
        _spi_dev(spi_dev),
        _spi_speed(spi_speed),
        _spi_mode(spi_mode),
        _spi_fd(-1),
        _gpio_fd(-1),
        _initialized(false) {
    }

    // Required virtual methods from RadioLibHal
    void init() override;
    void term() override;
    void pinMode(uint32_t pin, uint32_t mode) override;
    void digitalWrite(uint32_t pin, uint32_t value) override;
    uint32_t digitalRead(uint32_t pin) override;
    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override;
    void detachInterrupt(uint32_t interruptNum) override;
    void delay(RadioLibTime_t ms) override;
    void delayMicroseconds(RadioLibTime_t us) override;
    RadioLibTime_t millis() override;
    RadioLibTime_t micros() override;
    long pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout) override;
    void spiBegin() override;
    void spiBeginTransaction() override;
    void spiTransfer(uint8_t* out, size_t len, uint8_t* in) override;
    void spiEndTransaction() override;
    void spiEnd() override;
    void tone(uint32_t pin, unsigned int frequency, RadioLibTime_t duration = 0) override;
    void noTone(uint32_t pin) override;
    void yield() override;
    uint32_t pinToInterrupt(uint32_t pin) override;
    
  private:
    // Private members for Spresense hardware management
    const char* _spi_dev;           // SPI device path
    uint32_t _spi_speed;            // SPI speed
    uint32_t _spi_mode;             // SPI mode
    int _spi_fd;                    // SPI file descriptor
    int _gpio_fd;                   // GPIO file descriptor
    bool _initialized;              // Initialization flag
    RadioLibTime_t _start_time;     // System start time
    
    // Interrupt management
    struct {
        void (*callback)(void);
        uint32_t mode;
        bool enabled;
    } _interrupts[32];              // Support for up to 32 GPIO pins
    
    // Helper methods
    bool openSpiDevice();
    bool openGpioDevice();
    void closeSpiDevice();
    void closeGpioDevice();
    RadioLibTime_t getCurrentTime();
};

#endif // RADIOLIB_BUILD_SPRESENSE

#endif // SPRESENSE_NUTTX_HAL_H 