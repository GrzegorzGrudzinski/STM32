# STM32 Firmware Development

This repository contains a set of embedded drivers and a simple RTOS for the STM32F411 microcontroller. The code is written in C and is intended for bare-metal development using ARM toolchains.

## Project contents:

### stm32f411xx_periph_drivers
    Peripheral drivers (GPIO, SPI, ADC, TIM, etc.)

### Simple RTOS
    Minimal RTOS (work in progress - currently supports only round-robin scheduler)

### external_devices_drivers
    Drivers for various external devices. 
    Curently supports drivers for:
        - L3G4250D gyroscope


Note: This project is under active development. Some drivers and modules are not yet complete or fully functional.